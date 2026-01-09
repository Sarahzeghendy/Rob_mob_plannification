import math
import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist

import tf2_ros

from .map_utils import occgrid_to_2d, inflate_occupancy, world_to_cell, cell_to_world
from .astar import astar_occ_grid


def yaw_from_quat(q):
    """Extract yaw angle from quaternion"""
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class AStarPlannerNode(Node):
    def __init__(self):
        super().__init__("tb4_astar_planner")

        # Params
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("path_topic", "/astar_path")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        self.declare_parameter("robot_radius_m", 0.20)
        self.declare_parameter("occ_thresh", 50)
        self.declare_parameter("unknown_is_free", False)
        self.declare_parameter("allow_diag", True)

        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        
        # NEW: Use current robot pose as start automatically
        self.declare_parameter("auto_use_robot_pose", True)

        # Simple follower params
        self.declare_parameter("lookahead_m", 0.30)
        self.declare_parameter("v_max", 0.20)
        self.declare_parameter("w_max", 1.2)
        self.declare_parameter("yaw_kp", 1.8)
        self.declare_parameter("goal_tolerance_m", 0.12)

        # Subs
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.get_parameter("map_topic").value,
            self.on_map,
            10
        )
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

        # Pubs
        self.path_pub = self.create_publisher(Path, self.get_parameter("path_topic").value, 10)
        self.cmd_pub = self.create_publisher(Twist, self.get_parameter("cmd_vel_topic").value, 10)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State
        self.map_msg = None
        self.occ_2d = None
        self.inflated_2d = None

        self.start_world = None  # (x,y)
        self.goal_world = None   # (x,y)

        self.path_world = []     # list of (x,y)
        self.path_idx = 0

        self.timer = self.create_timer(0.05, self.on_control)  # 20 Hz

        self.get_logger().info("="*60)
        self.get_logger().info("tb4_astar_planner started!")
        self.get_logger().info(f"Listening to:")
        self.get_logger().info(f"   - Map: {self.get_parameter('map_topic').value}")
        self.get_logger().info(f"   - Goal: {self.get_parameter('goal_topic').value}")
        self.get_logger().info(f"   - Init pose: {self.get_parameter('initialpose_topic').value}")
        self.get_logger().info(f"Auto-use robot pose: {self.get_parameter('auto_use_robot_pose').value}")
        self.get_logger().info("="*60)

    def on_map(self, msg: OccupancyGrid):
        # Only log on significant map changes or first time
        if self.map_msg is None:
            self.get_logger().info(f"📍 First map received: {msg.info.width}x{msg.info.height}, res={msg.info.resolution:.3f}m")
        
        self.map_msg = msg
        W = msg.info.width
        H = msg.info.height
        self.occ_2d = occgrid_to_2d(msg.data, W, H)

        res = msg.info.resolution
        robot_radius = float(self.get_parameter("robot_radius_m").value)
        inflation_cells = int(robot_radius / res)

        occ_thresh = int(self.get_parameter("occ_thresh").value)
        self.inflated_2d = inflate_occupancy(self.occ_2d, inflation_cells, occ_thresh=occ_thresh)

        # Replan if we already have goal
        if self.goal_world is not None:
            self.try_plan()

    def on_initialpose(self, msg: PoseWithCovarianceStamped):
        self.start_world = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.get_logger().info(f"Manual start set: ({self.start_world[0]:.2f}, {self.start_world[1]:.2f})")
        self.try_plan()

    def on_goal(self, msg: PoseStamped):
        self.goal_world = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f"Goal received: ({self.goal_world[0]:.2f}, {self.goal_world[1]:.2f})")
        self.try_plan()

    def try_plan(self):
        if self.map_msg is None or self.inflated_2d is None:
            self.get_logger().warn("No map yet, cannot plan.")
            return
        
        if self.goal_world is None:
            self.get_logger().info("Waiting for goal...")
            return

        # Auto-use robot pose if enabled and no manual start set
        auto_use = bool(self.get_parameter("auto_use_robot_pose").value)
        if auto_use and self.start_world is None:
            pose = self.get_robot_pose()
            if pose is not None:
                x, y, _ = pose
                self.start_world = (x, y)
                self.get_logger().info(f"Using current robot pose as start: ({x:.2f}, {y:.2f})")
            else:
                self.get_logger().warn("Cannot get robot pose from TF. Waiting...")
                return
        
        if self.start_world is None:
            self.get_logger().info(" Waiting for start position (use '2D Pose Estimate' in RViz or enable auto_use_robot_pose)...")
            return

        origin_x = self.map_msg.info.origin.position.x
        origin_y = self.map_msg.info.origin.position.y
        res = self.map_msg.info.resolution

        start_rc = world_to_cell(self.start_world[0], self.start_world[1], origin_x, origin_y, res)
        goal_rc  = world_to_cell(self.goal_world[0],  self.goal_world[1],  origin_x, origin_y, res)

        self.get_logger().info(f" Planning from {start_rc} to {goal_rc}...")

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
        self.get_logger().info(f"Path planned: {len(self.path_world)} waypoints. Starting navigation! 🚀")

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
        """Get current robot pose from TF"""
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
            # Only log occasionally to avoid spam
            if not hasattr(self, '_tf_error_count'):
                self._tf_error_count = 0
            self._tf_error_count += 1
            if self._tf_error_count % 100 == 1:
                self.get_logger().warn(f" TF lookup failed: {str(e)[:100]}")
            return None

    def stop_robot(self):
        """Send zero velocity command"""
        t = Twist()
        self.cmd_pub.publish(t)

    def on_control(self):
        """Main control loop - called at 20Hz"""
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
            self.path_world = []  # Clear path
            self.start_world = None  # Reset start for next goal
            return

        lookahead = float(self.get_parameter("lookahead_m").value)

        # Advance path index while close to current waypoint
        while self.path_idx < len(self.path_world) - 1:
            wx, wy = self.path_world[self.path_idx]
            if math.hypot(wx - x, wy - y) < lookahead * 0.6:
                self.path_idx += 1
            else:
                break

        tx, ty = self.path_world[self.path_idx]

        # Heading control
        angle_to_target = math.atan2(ty - y, tx - x)
        ang_err = self.wrap_to_pi(angle_to_target - yaw)

        kp = float(self.get_parameter("yaw_kp").value)
        w = kp * ang_err

        w_max = float(self.get_parameter("w_max").value)
        w = max(-w_max, min(w_max, w))

        v_max = float(self.get_parameter("v_max").value)

        # Slow down if large angular error
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
    node = AStarPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()