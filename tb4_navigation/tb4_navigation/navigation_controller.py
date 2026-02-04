#!/usr/bin/env python3
"""
Navigation Controller Node 
Responsabilités :
- Coordonner planification et suivi de trajectoire
- Gérer la localisation manuelle 
- Publier TF map->odom
- Pré-calculer l'inflation 
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import String

import tf2_ros
from tf2_ros import TransformBroadcaster, TransformException
import numpy as np


from .utils.astar import astar_occ_grid
from .utils.map_utils import occgrid_to_2d, inflate_occupancy, world_to_cell, cell_to_world


def yaw_from_quat(q):
    """Extraire yaw d'un quaternion"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_from_yaw(yaw):
    """Convertir un angle yaw en quaternion"""
    return {
        'x': 0.0,
        'y': 0.0,
        'z': math.sin(yaw / 2.0),
        'w': math.cos(yaw / 2.0)
    }


class NavigationController(Node):
    """
    Contrôleur de navigation 
    """
    
    IDLE = "IDLE"
    PLANNING = "PLANNING"
    FOLLOWING = "FOLLOWING"
    GOAL_REACHED = "GOAL_REACHED"
    FAILED = "FAILED"
    
    def __init__(self):
        super().__init__('navigation_controller')
        
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('auto_use_robot_pose', True)
        
        self.declare_parameter('robot_radius_m', 0.20)
        self.declare_parameter('occ_thresh', 50)
        self.declare_parameter('allow_diag', True)  
        self.declare_parameter('unknown_is_free', False)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.map_to_odom_x = 0.0
        self.map_to_odom_y = 0.0
        self.map_to_odom_yaw = 0.0
        self.map_to_odom_initialized = False
        
        self.tf_timer = self.create_timer(0.05, self.publish_map_to_odom_tf)
        
        self.path_pub = self.create_publisher(Path, '/path', 10)
        self.state_pub = self.create_publisher(String, '~/state', 10)
        
        self.goal_sub = self.create_subscription(PoseStamped, '/nav_goal', self.on_goal, 10)
        
        self.init_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initialpose, 10)
        
        self.follower_status_sub = self.create_subscription(String, '/path_follower/status', self.on_follower_status, 10)

        self.inflated_map_pub = self.create_publisher(OccupancyGrid, '/inflated_map', 1 )

        
        map_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.map_msg = None
        self.occ_2d = None
        self.inflated_2d = None  
        
        self.start_world = None
        self.goal_world = None

        self.goal_yaw = None 
        
        self.state = self.IDLE
        
        self.get_logger().info("Navigation Controller started!")
        self.get_logger().info(f"   State: {self.state}")
        self.publish_state()
    
    def publish_map_to_odom_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.get_parameter("global_frame").value
        t.child_frame_id = self.get_parameter("odom_frame").value
        
        t.transform.translation.x = self.map_to_odom_x
        t.transform.translation.y = self.map_to_odom_y
        t.transform.translation.z = 0.0
        
        q = quat_from_yaw(self.map_to_odom_yaw)
        t.transform.rotation.x = q['x']
        t.transform.rotation.y = q['y']
        t.transform.rotation.z = q['z']
        t.transform.rotation.w = q['w']
        
        self.tf_broadcaster.sendTransform(t)
    
    def on_initialpose(self, msg: PoseWithCovarianceStamped):

        map_x = msg.pose.pose.position.x
        map_y = msg.pose.pose.position.y
        map_q = msg.pose.pose.orientation
        map_yaw = yaw_from_quat(map_q)
        
        self.get_logger().info(
            f"Initial pose received in map: ({map_x:.2f}, {map_y:.2f}, "
            f"{math.degrees(map_yaw):.1f}°)"
        )
        
        odom_frame = self.get_parameter("odom_frame").value
        base_frame = self.get_parameter("base_frame").value
        
        try:
            odom_to_base = self.tf_buffer.lookup_transform(
                odom_frame,
                base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            odom_x = odom_to_base.transform.translation.x
            odom_y = odom_to_base.transform.translation.y
            odom_q = odom_to_base.transform.rotation
            odom_yaw = yaw_from_quat(odom_q)
            
            self.get_logger().info(
                f"Robot pose in odom: ({odom_x:.2f}, {odom_y:.2f}, "
                f"{math.degrees(odom_yaw):.1f}°)"
            )
            
            self.map_to_odom_yaw = map_yaw - odom_yaw
            
            cos_theta = math.cos(self.map_to_odom_yaw)
            sin_theta = math.sin(self.map_to_odom_yaw)
            
            self.map_to_odom_x = map_x - (cos_theta * odom_x - sin_theta * odom_y)
            self.map_to_odom_y = map_y - (sin_theta * odom_x + cos_theta * odom_y)
            
            self.map_to_odom_initialized = True
            
            self.get_logger().info("Localisation mise à jour !")
            self.get_logger().info(
                f"  map->odom: ({self.map_to_odom_x:.2f}, {self.map_to_odom_y:.2f}, "
                f"{math.degrees(self.map_to_odom_yaw):.1f}°)"
            )
            
            self.start_world = (map_x, map_y)
            self.try_plan()
            
        except Exception as e:
            self.get_logger().error(f"Could not get odom->base_link transform: {e}")
            self.get_logger().warn("Make sure the robot is publishing odometry!")

    def publish_inflated_map(self):
        if self.map_msg is None or self.inflated_2d is None:
            return

        inflated_msg = OccupancyGrid()
        inflated_msg.header = self.map_msg.header
        inflated_msg.info = self.map_msg.info

        data = []
        h, w = self.inflated_2d.shape
        for r in range(h):
            for c in range(w):
                v = self.inflated_2d[r, c]
                if v < 0:
                    data.append(-1)
                else:
                    data.append(int(v))

        inflated_msg.data = data
        self.inflated_map_pub.publish(inflated_msg)

    
    def on_map(self, msg: OccupancyGrid):

        self.map_msg = msg
        
        self.occ_2d = occgrid_to_2d(msg.data, msg.info.width, msg.info.height)
        
        res = msg.info.resolution
        robot_radius = float(self.get_parameter("robot_radius_m").value)
        inflation_cells = int(robot_radius / res)
        occ_thresh = int(self.get_parameter("occ_thresh").value)
        
        self.inflated_2d = inflate_occupancy(
            self.occ_2d,
            inflation_cells,
            occ_thresh=occ_thresh
        )

        self.publish_inflated_map()
        
        if not hasattr(self, '_map_received'):
            self._map_received = True
            self.get_logger().info(
                f"Map received and inflated: {msg.info.width}x{msg.info.height} "
                f"@ {msg.info.resolution}m/cell"
            )
        
        if self.goal_world is not None and self.state == self.IDLE:
            self.try_plan()

    def on_goal(self, msg: PoseStamped):
        new_goal = (msg.pose.position.x, msg.pose.position.y)

        if self.goal_world is not None:
            dist = math.hypot(new_goal[0] - self.goal_world[0],
                            new_goal[1] - self.goal_world[1])
            if dist < 0.1:
                self.get_logger().debug("Same goal received, ignoring")
                return

        self.goal_world = new_goal

        self.goal_yaw = yaw_from_quat(msg.pose.orientation)

        self.get_logger().info(
            f"Goal received: ({self.goal_world[0]:.2f}, {self.goal_world[1]:.2f}) "
            f"yaw={math.degrees(self.goal_yaw):.1f}°"
        )
        self.try_plan()

    
    def try_plan(self):

        if self.map_msg is None or self.inflated_2d is None:
            self.get_logger().warn("No map loaded yet, cannot plan.")
            return
        
        if self.goal_world is None:
            self.get_logger().info("Waiting for goal...")
            return
        
        auto_use = bool(self.get_parameter("auto_use_robot_pose").value)
        if auto_use and self.start_world is None:
            pose = self.get_robot_pose()
            if pose is not None:
                x, y, _ = pose
                self.start_world = (x, y)
                self.get_logger().info(
                    f"Using current robot pose as start: ({x:.2f}, {y:.2f})"
                )
            else:
                self.get_logger().warn(
                    "Cannot get robot pose from TF. Use '2D Pose Estimate' first!"
                )
                return
        
        if self.start_world is None:
            self.get_logger().info(
                "Waiting for start position... Click '2D Pose Estimate' in RViz"
            )
            return
        
        origin_x = self.map_msg.info.origin.position.x
        origin_y = self.map_msg.info.origin.position.y
        res = self.map_msg.info.resolution
        
        start_rc = world_to_cell(
            self.start_world[0], self.start_world[1],
            origin_x, origin_y, res
        )
        goal_rc = world_to_cell(
            self.goal_world[0], self.goal_world[1],
            origin_x, origin_y, res
        )
        
        self.get_logger().info(f"Planning from {start_rc} to {goal_rc}...")
        
        allow_diag = bool(self.get_parameter("allow_diag").value)
        occ_thresh = int(self.get_parameter("occ_thresh").value)
        unknown_is_free = bool(self.get_parameter("unknown_is_free").value)
        
        try:
            path_rc = astar_occ_grid(
                self.inflated_2d,  
                start_rc,
                goal_rc,
                allow_diag=allow_diag,
                occ_thresh=occ_thresh,
                unknown_is_free=unknown_is_free
            )
        except Exception as e:
            self.get_logger().error(f"Planning error: {e}")
            self.set_state(self.FAILED)
            return
        
        if path_rc is None:
            self.get_logger().warn("No path found!")
            self.set_state(self.FAILED)
            return
        
        path_world = [
            cell_to_world(r, c, origin_x, origin_y, res)
            for (r, c) in path_rc
        ]
        
        self.publish_path(path_world)
        self.get_logger().info(f"Path planned: {len(path_world)} waypoints")
        
        self.set_state(self.FOLLOWING)
    
    def publish_path(self, path_world):

        if self.map_msg is None:
            return

        path_msg = Path()
        path_msg.header.frame_id = self.get_parameter("global_frame").value
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for i, (x, y) in enumerate(path_world):
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = 0.0

            if i == len(path_world) - 1 and self.goal_yaw is not None:
                q = quat_from_yaw(self.goal_yaw)
                ps.pose.orientation.x = q['x']
                ps.pose.orientation.y = q['y']
                ps.pose.orientation.z = q['z']
                ps.pose.orientation.w = q['w']
            else:
                ps.pose.orientation.w = 1.0

            path_msg.poses.append(ps)

        self.path_pub.publish(path_msg)


    
    def get_robot_pose(self):

        global_frame = self.get_parameter("global_frame").value
        base_frame = self.get_parameter("base_frame").value
        
        try:
            tf = self.tf_buffer.lookup_transform(
                global_frame,
                base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            q = tf.transform.rotation
            yaw = yaw_from_quat(q)
            return x, y, yaw
        except Exception as e:
            if not hasattr(self, '_tf_error_count'):
                self._tf_error_count = 0
            self._tf_error_count += 1
            if self._tf_error_count % 100 == 1:
                self.get_logger().warn(f"TF lookup failed: {str(e)[:100]}")
            return None
    
    def on_follower_status(self, msg: String):

        follower_status = msg.data
        
        if follower_status == "GOAL_REACHED":
            self.get_logger().info("Goal reached!")
            self.set_state(self.GOAL_REACHED)
            # Reset pour prochaine navigation
            self.start_world = None
            self.goal_world = None
            self.goal_yaw = None

        elif follower_status == "FAILED":
            self.get_logger().warn("Path follower failed")
            self.set_state(self.FAILED)
    
    def set_state(self, new_state):

        if new_state != self.state:
            self.get_logger().info(f"State: {self.state} → {new_state}")
            self.state = new_state
            self.publish_state()
    
    def publish_state(self):

        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)


def main():
    rclpy.init()
    node = NavigationController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()