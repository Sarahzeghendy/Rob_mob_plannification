import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class MyTeleopJoy(Node):
    def __init__(self):
        super().__init__("my_teleop_joy_node")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.joy_sub = self.create_subscription(
            Joy,
            "/joy",
            self.joy_callback,
            10
        )

    def joy_callback(self, msg: Joy):

        linear = msg.axes[1] * 2.0
        angular = msg.axes[0] * 2.5

        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = MyTeleopJoy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
