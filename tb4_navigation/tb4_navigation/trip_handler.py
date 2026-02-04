#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose
from std_msgs.msg import String

import tf2_ros
from tf2_ros import TransformException


class TripHandler(Node):
    IDLE = "IDLE"
    GOING_TO_GOAL = "GOING_TO_GOAL"
    WAITING_AT_GOAL = "WAITING_AT_GOAL"
    RETURNING_TO_START = "RETURNING_TO_START"
    MISSION_COMPLETE = "MISSION_COMPLETE"

    def __init__(self):
        super().__init__('trip_handler')

        self.declare_parameter('return_to_start_enabled', True)
        self.declare_parameter('wait_before_return_s', 20.0)
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        self.nav_goal_pub = self.create_publisher(PoseStamped, '/nav_goal', 10)
        self.status_pub = self.create_publisher(String, '~/trip_status', 10)

        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.on_user_goal, 10)
        self.nav_state_sub = self.create_subscription(String, '/navigation_controller/state', self.on_nav_state, 10)
        self.init_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initial_pose, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.state = self.IDLE
        self.start_pose: Pose | None = None
        self.start_pose_locked = False

        self.wait_timer = None

        self.get_logger().info("Trip Handler started!")
        self.get_logger().info(f"  Return-to-start: {self.get_parameter('return_to_start_enabled').value}")
        self.get_logger().info(f"  Wait: {self.get_parameter('wait_before_return_s').value}s")
        self.publish_status()

    def publish_status(self):
        msg = String()
        msg.data = self.state
        self.status_pub.publish(msg)

    def set_state(self, new_state: str):
        if new_state != self.state:
            self.get_logger().info(f"Trip state: {self.state} → {new_state}")
            self.state = new_state
            self.publish_status()

    def cancel_wait_timer(self):
        if self.wait_timer is not None:
            self.wait_timer.cancel()
            self.wait_timer = None

    def get_current_pose_in_global(self) -> Pose | None:
        global_frame = self.get_parameter('global_frame').value
        base_frame = self.get_parameter('base_frame').value
        try:
            tf = self.tf_buffer.lookup_transform(
                global_frame,
                base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            p = Pose()
            p.position.x = float(tf.transform.translation.x)
            p.position.y = float(tf.transform.translation.y)
            p.position.z = 0.0
            p.orientation = tf.transform.rotation
            return p
        except TransformException as e:
            self.get_logger().warn(f"TF unavailable ({global_frame}->{base_frame}): {str(e)[:140]}")
            return None

    def on_initial_pose(self, msg: PoseWithCovarianceStamped):
        if self.start_pose_locked:
            return
        self.start_pose = msg.pose.pose
        self.get_logger().info(
            f"Start pose stored (/initialpose): ({self.start_pose.position.x:.2f}, {self.start_pose.position.y:.2f})"
        )

    def on_user_goal(self, msg: PoseStamped):
        if self.state == self.WAITING_AT_GOAL:
            self.get_logger().info("New goal received during waiting -> cancel return and go.")
            self.cancel_wait_timer()
            self.set_state(self.GOING_TO_GOAL)
            self.nav_goal_pub.publish(msg)
            return

        if self.state not in (self.IDLE, self.MISSION_COMPLETE):
            self.get_logger().warn(f"Trip in progress (state: {self.state}), ignoring new goal")
            return

       
        if self.start_pose is None:
            tf_pose = self.get_current_pose_in_global()
            if tf_pose is not None:
                self.start_pose = tf_pose
                self.get_logger().info(
                    f"Start pose stored (from TF): ({self.start_pose.position.x:.2f}, {self.start_pose.position.y:.2f})"
                )
            else:
                self.get_logger().warn("No start pose available -> will go to goal, but auto-return may be disabled.")

        self.start_pose_locked = (self.start_pose is not None)

        self.set_state(self.GOING_TO_GOAL)
        self.nav_goal_pub.publish(msg)
        self.get_logger().info(f"Goal forwarded to /nav_goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")

    def on_nav_state(self, msg: String):
        nav_state = msg.data

        if nav_state == "GOAL_REACHED":
            if self.state == self.GOING_TO_GOAL:
                self.handle_goal_reached()
            elif self.state == self.RETURNING_TO_START:
                self.handle_return_complete()

        elif nav_state == "FAILED":
            self.get_logger().error("Navigation failed!")
            self.cancel_wait_timer()
            self.set_state(self.IDLE)
            self.start_pose_locked = False

    def handle_goal_reached(self):
        self.get_logger().info("Goal reached!")

        return_enabled = bool(self.get_parameter('return_to_start_enabled').value)
        if not return_enabled:
            self.set_state(self.MISSION_COMPLETE)
            self.start_pose_locked = False
            return

        if self.start_pose is None:
            self.get_logger().warn("No start_pose stored -> cannot return to start. Mission complete.")
            self.set_state(self.MISSION_COMPLETE)
            self.start_pose_locked = False
            return

        wait_s = float(self.get_parameter('wait_before_return_s').value)
        self.set_state(self.WAITING_AT_GOAL)

        self.cancel_wait_timer()
        self.wait_timer = self.create_timer(wait_s, self.on_wait_timeout)
        self.get_logger().info(f"Waiting {wait_s:.0f}s for a new goal before returning to start...")

    def on_wait_timeout(self):
        self.cancel_wait_timer()
        if self.state != self.WAITING_AT_GOAL:
            return
        self.get_logger().info("No new goal received -> returning to start.")
        self.return_to_start()

    def return_to_start(self):
        if self.start_pose is None:
            self.get_logger().warn("Return requested but no start_pose stored.")
            self.set_state(self.MISSION_COMPLETE)
            self.start_pose_locked = False
            return

        self.set_state(self.RETURNING_TO_START)

        return_goal = PoseStamped()
        return_goal.header.frame_id = self.get_parameter('global_frame').value
        return_goal.header.stamp = self.get_clock().now().to_msg()
        return_goal.pose = self.start_pose

        self.nav_goal_pub.publish(return_goal)
        self.get_logger().info(
            f"Returning to start: ({self.start_pose.position.x:.2f}, {self.start_pose.position.y:.2f})"
        )

    def handle_return_complete(self):
        self.set_state(self.MISSION_COMPLETE)
        self.get_logger().info("Return to start complete!")
        self.start_pose_locked = False


def main(args=None):
    rclpy.init(args=args)
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
