from launch import LaunchDescription
from launch_ros.actions import Node 
def generate_launch_description():
    return LaunchDescription([

        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_linux_node'
        ),

        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim_node'
        ),
        Node(
            package='my_teleop_joy',
            executable='my_teleop_joy_node',
            name='my_teleop_joy_node',
            namespace='/turtlesim1/cmd_vel',
        )
    ])









    