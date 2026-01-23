#!/usr/bin/env python3
"""
Planificateur A* qui utilise une carte sauvegardée
Charge une carte depuis un fichier YAML/PGM au lieu d'attendre le topic /map
Avec localisation manuelle (publie le TF map->odom)
"""

import math
import numpy as np
import yaml
from PIL import Image
import os

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, TransformStamped

import tf2_ros
from tf2_ros import TransformBroadcaster

from .map_utils import occgrid_to_2d, inflate_occupancy, world_to_cell, cell_to_world
from .astar import astar_occ_grid


def yaw_from_quat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_from_yaw(yaw):
    """Convertir un angle yaw en quaternion"""
    return {
        'x': 0.0,
        'y': 0.0,
        'z': math.sin(yaw / 2.0),
        'w': math.cos(yaw / 2.0)
    }


class AStarPlannerWithSavedMap(Node):
    def __init__(self):
        super().__init__("tb4_astar_planner")

        self.declare_parameter("map_yaml_path", "")  
        self.declare_parameter("use_topic_map", False)  
        
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("path_topic", "/astar_path")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("publish_map", True)  

        self.declare_parameter("robot_radius_m", 0.20)
        self.declare_parameter("occ_thresh", 50)
        self.declare_parameter("unknown_is_free", False)
        self.declare_parameter("allow_diag", True)

        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")  
        self.declare_parameter("auto_use_robot_pose", True)

        self.declare_parameter("lookahead_m", 0.30)
        self.declare_parameter("v_max", 0.20)
        self.declare_parameter("w_max", 1.2)
        self.declare_parameter("yaw_kp", 1.8)
        self.declare_parameter("goal_tolerance_m", 0.12)

        # Publishers
        self.path_pub = self.create_publisher(Path, self.get_parameter("path_topic").value, 10)
        self.cmd_pub = self.create_publisher(Twist, self.get_parameter("cmd_vel_topic").value, 10)
        
        if self.get_parameter("publish_map").value:
            self.map_pub = self.create_publisher(OccupancyGrid, "/map_static", 10)

        self.init_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("initialpose_topic").value,
            self.on_initialpose,
            10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.get_parameter("goal_topic").value,
            self.on_goal,
            10
        )
        
        if self.get_parameter("use_topic_map").value:
            self.map_sub = self.create_subscription(
                OccupancyGrid,
                self.get_parameter("map_topic").value,
                self.on_map,
                10
            )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.map_to_odom_x = 0.0
        self.map_to_odom_y = 0.0
        self.map_to_odom_yaw = 0.0
        self.map_to_odom_initialized = False
        
        self.tf_timer = self.create_timer(0.05, self.publish_map_to_odom_tf)  

        self.map_msg = None
        self.occ_2d = None
        self.inflated_2d = None

        self.start_world = None
        self.goal_world = None

        self.path_world = []
        self.path_idx = 0

        self.timer = self.create_timer(0.05, self.on_control)
        
        if self.get_parameter("publish_map").value:
            self.map_pub_timer = self.create_timer(1.0, self.publish_loaded_map)

        self.get_logger().info("A* Planner with Saved Map started!")
        self.get_logger().info("Localisation manuelle activée (publie map->odom)")
        
        map_yaml_path = self.get_parameter("map_yaml_path").value
        if map_yaml_path:
            self.load_map_from_file(map_yaml_path)
        else:
            self.get_logger().warn("No map file specified. Set 'map_yaml_path' parameter!")
            self.get_logger().info("Example: ros2 run tb4_astar_planner planner_node --ros-args -p map_yaml_path:=/path/to/map.yaml")
        
        self.get_logger().info("="*60)

    def publish_map_to_odom_tf(self):
        """Publier la transformation map->odom"""
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

    def load_map_from_file(self, yaml_path):
        try:
            self.get_logger().info(f"Loading map from: {yaml_path}")
            
            with open(yaml_path, 'r') as f:
                map_metadata = yaml.safe_load(f)
            
            resolution = map_metadata['resolution']
            origin = map_metadata['origin']  
            negate = map_metadata.get('negate', 0)
            occupied_thresh = map_metadata.get('occupied_thresh', 0.65)
            free_thresh = map_metadata.get('free_thresh', 0.196)
            
            image_path = map_metadata['image']
            if not image_path.startswith('/'):
                map_dir = os.path.dirname(yaml_path)
                image_path = os.path.join(map_dir, image_path)
            
            self.get_logger().info(f"Loading image: {image_path}")
            
            img = Image.open(image_path)
            img_array = np.array(img)
            
            height, width = img_array.shape
            
            self.map_msg = OccupancyGrid()
            self.map_msg.header.frame_id = self.get_parameter("global_frame").value
            self.map_msg.header.stamp = self.get_clock().now().to_msg()
            
            self.map_msg.info.resolution = resolution
            self.map_msg.info.width = width
            self.map_msg.info.height = height
            self.map_msg.info.origin.position.x = origin[0]
            self.map_msg.info.origin.position.y = origin[1]
            self.map_msg.info.origin.position.z = 0.0
            self.map_msg.info.origin.orientation.w = 1.0
            
            data = []
            for row in reversed(img_array):  
                for pixel in row:
                    if pixel == 254:  # Free
                        data.append(0)
                    elif pixel == 0:  # Occupied
                        data.append(100)
                    elif pixel == 205:  # Unknown
                        data.append(-1)
                    else:
                        # Interpoler
                        normalized = pixel / 255.0
                        if normalized > occupied_thresh:
                            data.append(0)
                        elif normalized < free_thresh:
                            data.append(100)
                        else:
                            occ = int((1.0 - normalized) * 100)
                            data.append(occ)
            
            self.map_msg.data = data
            
            self.occ_2d = occgrid_to_2d(data, width, height)
            
            # Inflater les obstacles
            res = resolution
            robot_radius = float(self.get_parameter("robot_radius_m").value)
            inflation_cells = int(robot_radius / res)
            occ_thresh = int(self.get_parameter("occ_thresh").value)
            
            self.inflated_2d = inflate_occupancy(self.occ_2d, inflation_cells, occ_thresh=occ_thresh)
            
            self.get_logger().info(f"Map loaded successfully!")
            self.get_logger().info(f"   Size: {width}x{height} pixels")
            self.get_logger().info(f"   Resolution: {resolution} m/pixel")
            self.get_logger().info(f"   Origin: ({origin[0]:.2f}, {origin[1]:.2f})")
            
            if self.goal_world is not None:
                self.try_plan()
                
        except Exception as e:
            self.get_logger().error(f"Failed to load map: {e}")

    def publish_loaded_map(self):
        if self.map_msg is not None:
            self.map_msg.header.stamp = self.get_clock().now().to_msg()
            self.map_pub.publish(self.map_msg)

    def on_map(self, msg: OccupancyGrid):
        self.map_msg = msg
        self.occ_2d = occgrid_to_2d(msg.data, msg.info.width, msg.info.height)
        
        res = msg.info.resolution
        robot_radius = float(self.get_parameter("robot_radius_m").value)
        inflation_cells = int(robot_radius / res)
        occ_thresh = int(self.get_parameter("occ_thresh").value)
        
        self.inflated_2d = inflate_occupancy(self.occ_2d, inflation_cells, occ_thresh=occ_thresh)
        
        if self.goal_world is not None:
            self.try_plan()

    def on_initialpose(self, msg: PoseWithCovarianceStamped):
        """
        Callback pour /initialpose (2D Pose Estimate dans RViz)
        Calcule et met à jour le TF map->odom
        """
        map_x = msg.pose.pose.position.x
        map_y = msg.pose.pose.position.y
        map_q = msg.pose.pose.orientation
        map_yaw = yaw_from_quat(map_q)
        
        self.get_logger().info(f"Initial pose received in map: ({map_x:.2f}, {map_y:.2f}, {math.degrees(map_yaw):.1f}°)")
        
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
            
            self.get_logger().info(f"Robot pose in odom: ({odom_x:.2f}, {odom_y:.2f}, {math.degrees(odom_yaw):.1f}°)")
            
            # Calculer map->odom
            # On veut que : map->odom + odom->base_link = map->base_link
            # Donc : map->odom = map->base_link - odom->base_link
            
            # Pour la position :
            # map_to_odom * odom_to_base = map_to_base
            # En 2D avec rotation :
            # [map_x]   [cos(θ) -sin(θ)] [odom_x]   [tx]
            # [map_y] = [sin(θ)  cos(θ)] [odom_y] + [ty]
            #
            # On cherche tx, ty, θ tels que :
            # map_to_base = Rot(map_to_odom_yaw) * odom_to_base + trans(map_to_odom_x, map_to_odom_y)
            
            self.map_to_odom_yaw = map_yaw - odom_yaw
            
            # Calcul de la translation
            # map_x = map_to_odom_x + cos(map_to_odom_yaw) * odom_x - sin(map_to_odom_yaw) * odom_y
            # map_y = map_to_odom_y + sin(map_to_odom_yaw) * odom_x + cos(map_to_odom_yaw) * odom_y
            
            cos_theta = math.cos(self.map_to_odom_yaw)
            sin_theta = math.sin(self.map_to_odom_yaw)
            
            self.map_to_odom_x = map_x - (cos_theta * odom_x - sin_theta * odom_y)
            self.map_to_odom_y = map_y - (sin_theta * odom_x + cos_theta * odom_y)
            
            self.map_to_odom_initialized = True
            
            self.get_logger().info(f"Localisation mise à jour !")
            self.get_logger().info(f"  map->odom: ({self.map_to_odom_x:.2f}, {self.map_to_odom_y:.2f}, {math.degrees(self.map_to_odom_yaw):.1f}°)")
            
            self.start_world = (map_x, map_y)
            self.try_plan()
            
        except Exception as e:
            self.get_logger().error(f"Could not get odom->base_link transform: {e}")
            self.get_logger().warn("Make sure the robot is publishing odometry!")

    def on_goal(self, msg: PoseStamped):
        new_goal = (msg.pose.position.x, msg.pose.position.y)
        
        if self.goal_world is not None:
            dist = math.hypot(new_goal[0] - self.goal_world[0], 
                            new_goal[1] - self.goal_world[1])
            if dist < 0.1:  
                self.get_logger().debug("Same goal received, ignoring")
                return
        
        self.goal_world = new_goal
        self.get_logger().info(f"Goal received: ({self.goal_world[0]:.2f}, {self.goal_world[1]:.2f})")
        self.try_plan()

    def try_plan(self):
        if self.map_msg is None or self.inflated_2d is None:
            self.get_logger().warn("No map loaded yet, cannot plan.")
            self.get_logger().info("Set the 'map_yaml_path' parameter to load a saved map")
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
                self.get_logger().info(f"Using current robot pose as start: ({x:.2f}, {y:.2f})")
            else:
                self.get_logger().warn("Cannot get robot pose from TF. Use '2D Pose Estimate' first!")
                return
        
        if self.start_world is None:
            self.get_logger().info("Waiting for start position... Click '2D Pose Estimate' in RViz")
            return

        origin_x = self.map_msg.info.origin.position.x
        origin_y = self.map_msg.info.origin.position.y
        res = self.map_msg.info.resolution

        start_rc = world_to_cell(self.start_world[0], self.start_world[1], origin_x, origin_y, res)
        goal_rc = world_to_cell(self.goal_world[0], self.goal_world[1], origin_x, origin_y, res)

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
            self.stop_robot()
            return

        if path_rc is None:
            self.get_logger().warn("No path found!")
            self.path_world = []
            self.path_idx = 0
            self.stop_robot()
            return

        # Convert to world coordinates
        self.path_world = [cell_to_world(r, c, origin_x, origin_y, res) for (r, c) in path_rc]
        self.path_idx = 0

        self.publish_path()
        self.get_logger().info(f"Path planned: {len(self.path_world)} waypoints")

    def publish_path(self):
        if self.map_msg is None:
            return
        path_msg = Path()
        path_msg.header.frame_id = self.get_parameter("global_frame").value
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for (x, y) in self.path_world:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = 0.0
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

    def stop_robot(self):
        t = Twist()
        self.cmd_pub.publish(t)

    def on_control(self):
        if not self.path_world:
            return

        pose = self.get_robot_pose()
        if pose is None:
            return

        x, y, yaw = pose
        goal_tol = float(self.get_parameter("goal_tolerance_m").value)

        # Check if goal reached
        gx, gy = self.path_world[-1]
        dist_to_goal = math.hypot(gx - x, gy - y)
        
        if dist_to_goal < goal_tol:
            self.stop_robot()
            self.get_logger().info(f"Goal reached! Distance: {dist_to_goal:.3f}m")
            self.path_world = []
            self.start_world = None
            self.goal_world = None 
            self.path_idx = 0    
            return

        lookahead = float(self.get_parameter("lookahead_m").value)

        # Advance path index
        if self.path_idx < len(self.path_world) - 1:
            wx, wy = self.path_world[self.path_idx]
            dist_to_current = math.hypot(wx - x, wy - y)
            
            if dist_to_current < lookahead:
                self.path_idx += 1

        tx, ty = self.path_world[self.path_idx]

        # Heading control
        angle_to_target = math.atan2(ty - y, tx - x)
        ang_err = self.wrap_to_pi(angle_to_target - yaw)

        kp = float(self.get_parameter("yaw_kp").value)
        w = kp * ang_err

        w_max = float(self.get_parameter("w_max").value)
        w = max(-w_max, min(w_max, w))

        v_max = float(self.get_parameter("v_max").value)
        v = v_max * max(0.0, 1.0 - abs(ang_err) / 1.2)

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

    @staticmethod
    def wrap_to_pi(a):
        """Wrap angle to [-pi, pi]"""
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a


def main():
    rclpy.init()
    node = AStarPlannerWithSavedMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()