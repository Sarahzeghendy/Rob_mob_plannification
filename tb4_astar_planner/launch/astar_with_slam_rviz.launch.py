from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    # RViz config
    pkg_share = get_package_share_directory("my_teleop_joy")
    rviz_config = os.path.join(pkg_share, "rviz", "turtlebot_config.rviz")

    # Gazebo simulation
    tb3_pkg = get_package_share_directory("turtlebot3_gazebo")
    tb3_launch = os.path.join(tb3_pkg, "launch", "turtlebot3_dqn_stage2.launch.py")

    # SLAM toolbox
    slam_pkg = get_package_share_directory("slam_toolbox")
    slam_launch = os.path.join(slam_pkg, "launch", "online_async_launch.py")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use sim time (Gazebo). Set false on real robot."
        ),

        # Launch Gazebo simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tb3_launch),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        ),

     
        Node(
            package="tb4_astar_planner",
            executable="planner_node",
            name="tb4_astar_planner",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,

                # Topics
                "map_topic": "/map",
                "initialpose_topic": "/initialpose",
                "goal_topic": "/goal_pose",
                "path_topic": "/astar_path",
                "cmd_vel_topic": "/cmd_vel",

                # Frames
                "global_frame": "map",
                "base_frame": "base_footprint",

                # Planning parameters
                "robot_radius_m": 0.20,
                "occ_thresh": 50,
                "allow_diag": True,
                "unknown_is_free": False,

                # Control parameters
                "lookahead_m": 0.30,
                "v_max": 0.20,
                "w_max": 1.2,
                "yaw_kp": 1.8,
                "goal_tolerance_m": 0.12,
            }],
        ),
    ])