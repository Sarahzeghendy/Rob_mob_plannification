from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml_file = LaunchConfiguration("map_yaml_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time"
        ),
        
        DeclareLaunchArgument(
            "map_yaml_file",
            default_value="/home/sarah/robmob_ws/src/tb4_astar_planner/map/test_map1.yaml",
            description="Path to map YAML file"
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_tf",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            parameters=[{"use_sim_time": use_sim_time}],
        ),

        Node(
            package="tb4_navigation",
            executable="map_manager",
            name="map_manager",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "map_yaml_file": map_yaml_file,
                "publish_rate": 1.0,
                "global_frame": "map",
            }]
        ),


        Node(
            package="tb4_navigation",
            executable="path_follower",
            name="path_follower",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "global_frame": "map",
                "base_frame": "base_link",
                "lookahead_distance": 0.35,
                "max_linear_vel": 0.12,
                "max_angular_vel": 0.6,
                "yaw_gain": 1.5,
                "goal_tolerance": 0.15,
                "control_frequency": 20.0,
            }]
        ),

        Node(
            package="tb4_navigation",
            executable="navigation_controller",
            name="navigation_controller",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "global_frame": "map",
                "base_frame": "base_link",
                "auto_replan_on_failure": True,
                "max_replan_attempts": 3,
            }]
        ),

        Node(
            package='tb4_navigation',
            executable='trip_handler',
            name='trip_handler',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'return_to_start_enabled': True,
                'wait_before_return_s': 20.0,
                'global_frame': 'map',
                'base_frame': 'base_footprint',  
            }]
        ),

        Node(
            package="tb4_navigation",
            executable="goal_input",
            name="goal_input",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "goal_topic": "/goal_pose",
                "global_frame": "map",
            }]
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])