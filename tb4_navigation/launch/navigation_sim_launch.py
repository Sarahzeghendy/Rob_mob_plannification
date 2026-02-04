from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml_file = LaunchConfiguration("map_yaml_file")

    tb3_gazebo_pkg = get_package_share_directory("turtlebot3_gazebo")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time"
        ),
        
        DeclareLaunchArgument(
            "map_yaml_file",
            default_value="/home/evinia/robmob_ws/src/Rob_mob_plannification/tb4_navigation/map/my_map.yaml",
            description="Path to map YAML file"
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_gazebo_pkg, "launch", "turtlebot3_dqn_stage2.launch.py")
            ),
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
                "base_frame": "base_footprint",
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
                "base_frame": "base_footprint",
                "odom_frame": "odom",
                "auto_replan_on_failure": True,
                "max_replan_attempts": 3,
                "auto_use_robot_pose": True,
                
                "robot_radius_m": 0.3,
                "occ_thresh": 50,
                "allow_diagonal": True,
                "unknown_is_free": False,
                "inflation_radius_m": 0.8,
            }]
        ),

        Node(
            package='tb4_navigation',
            executable='trip_handler',
            name='trip_handler',
            parameters=[{
                'return_to_start_enabled': True,
                'global_frame': 'map'
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