#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String
import math


class TripHandler(Node):
    
    IDLE = "IDLE"
    GOING_TO_GOAL = "GOING_TO_GOAL"
    RETURNING_TO_START = "RETURNING_TO_START"
    MISSION_COMPLETE = "MISSION_COMPLETE"
    
    def __init__(self):
        super().__init__('trip_handler')
        
        self.declare_parameter('return_to_start_enabled', True)
        self.declare_parameter('global_frame', 'map')
        
        self.nav_goal_pub = self.create_publisher(PoseStamped, '/nav_goal', 10)
        self.status_pub = self.create_publisher(String, '~/trip_status', 10)
        
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.on_user_goal, 10)
        self.nav_state_sub = self.create_subscription(String, '/navigation_controller/state', self.on_nav_state, 10)
        self.init_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initial_pose, 10)
        
        self.state = self.IDLE
        self.start_pose = None
        self.current_goal = None
        
        self.get_logger().info("Trip Handler started!")
        self.get_logger().info(f"  Return-to-start: {self.get_parameter('return_to_start_enabled').value}")
    
    def on_initial_pose(self, msg: PoseWithCovarianceStamped):
        self.start_pose = msg.pose.pose
        self.get_logger().info(
            f"Start pose stored: ({self.start_pose.position.x:.2f}, "
            f"{self.start_pose.position.y:.2f})"
        )
    
    def on_user_goal(self, msg: PoseStamped):
        if self.start_pose is None:
            self.get_logger().warn("No start pose set! Use '2D Pose Estimate' in RViz first.")
            return
        
        if self.state != self.IDLE and self.state != self.MISSION_COMPLETE:
            self.get_logger().warn(f"Trip in progress (state: {self.state}), ignoring new goal")
            return
        
        self.current_goal = msg
        self.set_state(self.GOING_TO_GOAL)
        
        # Forward goal to navigation controller
        self.nav_goal_pub.publish(msg)
        self.get_logger().info(f"Trip started to ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")
    
    def on_nav_state(self, msg: String):
        nav_state = msg.data
        
        if nav_state == "GOAL_REACHED":
            if self.state == self.GOING_TO_GOAL:
                self.handle_goal_reached()
            elif self.state == self.RETURNING_TO_START:
                self.handle_return_complete()
        
        elif nav_state == "FAILED":
            self.get_logger().error("Navigation failed!")
            self.set_state(self.IDLE)
    
    def handle_goal_reached(self):
        self.get_logger().info("Goal reached!")
        return_enabled = self.get_parameter('return_to_start_enabled').value
        
        if return_enabled and self.start_pose is not None:
            self.return_to_start()
        else:
            self.set_state(self.MISSION_COMPLETE)
            self.get_logger().info("Mission complete!")
    
    def return_to_start(self):
        self.set_state(self.RETURNING_TO_START)
        
        # Create goal from stored start pose
        return_goal = PoseStamped()
        return_goal.header.frame_id = self.get_parameter('global_frame').value
        return_goal.header.stamp = self.get_clock().now().to_msg()
        return_goal.pose = self.start_pose
        
        self.nav_goal_pub.publish(return_goal)
        self.get_logger().info(f"Returning to start: ({self.start_pose.position.x:.2f}, {self.start_pose.position.y:.2f})")
    
    def handle_return_complete(self):
        self.set_state(self.MISSION_COMPLETE)
        self.get_logger().info("Return to start complete!")
    
    def set_state(self, new_state):
        if new_state != self.state:
            self.get_logger().info(f"Trip state: {self.state} → {new_state}")
            self.state = new_state
            self.publish_status()
    
    def publish_status(self):
        msg = String()
        msg.data = self.state
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = TripHandler()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
