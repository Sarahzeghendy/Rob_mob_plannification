#!/usr/bin/env python3
"""
Loi de commande 
"""

import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import tf2_ros
from tf2_ros import TransformException


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class PathFollower(Node):
  
    IDLE = "IDLE"
    FOLLOWING = "FOLLOWING"
    GOAL_REACHED = "GOAL_REACHED"
    FAILED = "FAILED"
    
    def __init__(self):
        super().__init__('path_follower')
        
        self.declare_parameter('lookahead_distance', 0.30)
        self.declare_parameter('max_linear_vel', 0.20)
        self.declare_parameter('max_angular_vel', 1.2)
        self.declare_parameter('yaw_gain', 1.8)
        self.declare_parameter('goal_tolerance', 0.12)
        self.declare_parameter('control_frequency', 20.0)
        
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        self.declare_parameter('yaw_tolerance', 0.15)   
        self.declare_parameter('final_yaw_gain', 2.0)
        self.goal_yaw = None  

        
        self.path_sub = self.create_subscription(
            Path,
            '/path',
            self.on_path_received,
            10
        )
        
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            '~/status',
            10
        )
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.current_path = []
        self.path_index = 0
        self.state = self.IDLE
        
        control_freq = self.get_parameter('control_frequency').value
        self.control_timer = self.create_timer(
            1.0 / control_freq,
            self.control_loop
        )
        
        self.stop_robot()
        
        self.get_logger().info("Path Follower started!")
        self.get_logger().info(f"   Lookahead: {self.get_parameter('lookahead_distance').value}m")
        self.get_logger().info(f"   Max velocities: v={self.get_parameter('max_linear_vel').value} m/s, "
                             f"w={self.get_parameter('max_angular_vel').value} rad/s")
        self.publish_status(self.IDLE)
    
    def on_path_received(self, msg: Path):
        if len(msg.poses) < 2:
            self.get_logger().warn("Path too short (< 2 waypoints), ignoring")
            return
        
        self.current_path = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in msg.poses
        ]
        
        self.path_index = 0
        self.state = self.FOLLOWING

        last_q = msg.poses[-1].pose.orientation
        self.goal_yaw = yaw_from_quaternion(last_q)

        
        self.get_logger().info(f"✓ New path received: {len(self.current_path)} waypoints")
        self.publish_status(self.FOLLOWING)
    
    def control_loop(self):

        
        if not self.current_path:
            if self.state == self.IDLE:
                self.stop_robot()
            return
        
        if self.state != self.FOLLOWING:
            self.stop_robot()
            return
        
        pose = self.get_robot_pose()
        if pose is None:
            return
        
        x, y, yaw = pose
        
        goal_tol = float(self.get_parameter('goal_tolerance').value)
        
        gx, gy = self.current_path[-1]
        dist_to_goal = math.hypot(gx - x, gy - y)
        
        if dist_to_goal < goal_tol:
            yaw_tol = float(self.get_parameter('yaw_tolerance').value)

            if self.goal_yaw is not None:
                yaw_err = wrap_angle(self.goal_yaw - yaw)

                if abs(yaw_err) > yaw_tol:
                    kp = float(self.get_parameter('final_yaw_gain').value)
                    w = kp * yaw_err

                    w_max = float(self.get_parameter('max_angular_vel').value)
                    w = max(-w_max, min(w_max, w))

                    cmd = Twist()
                    cmd.linear.x = 0.0
                    cmd.angular.z = float(w)
                    self.cmd_pub.publish(cmd)
                    return

            self.stop_robot()
            self.get_logger().info(f"Goal reached (XY + yaw)! Distance: {dist_to_goal:.3f}m")

            self.state = self.GOAL_REACHED
            self.publish_status(self.GOAL_REACHED)

            self.current_path = []
            self.path_index = 0
            self.goal_yaw = None
            return

        
        lookahead = float(self.get_parameter('lookahead_distance').value)
        
        if self.path_index < len(self.current_path) - 1:
            wx, wy = self.current_path[self.path_index]
            dist_to_current = math.hypot(wx - x, wy - y)
            
            if dist_to_current < lookahead:
                self.path_index += 1
        
        tx, ty = self.current_path[self.path_index]
        
        angle_to_target = math.atan2(ty - y, tx - x)
        ang_err = wrap_angle(angle_to_target - yaw)
        
        kp = float(self.get_parameter('yaw_gain').value)
        w = kp * ang_err
        
        w_max = float(self.get_parameter('max_angular_vel').value)
        w = max(-w_max, min(w_max, w))
        
        v_max = float(self.get_parameter('max_linear_vel').value)
        v = v_max * max(0.0, 1.0 - abs(ang_err) / 1.2)
        
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)
    
    def get_robot_pose(self):
  
        global_frame = self.get_parameter('global_frame').value
        base_frame = self.get_parameter('base_frame').value
        
        try:
            transform = self.tf_buffer.lookup_transform(
                global_frame,
                base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            yaw = yaw_from_quaternion(transform.transform.rotation)
            
            return (x, y, yaw)
        
        except TransformException:
            return None
    
    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)
    
    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = PathFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()