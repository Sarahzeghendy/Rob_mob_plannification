from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml_file = LaunchConfiguration("map_yaml_file")

    tb3_gazebo_pkg = get_package_share_directory("turtlebot3_gazebo")
    tb4_planner_pkg = get_package_share_directory("tb4_astar_planner")
    
    rviz_config_file = ""
    possible_rviz = [
        os.path.join(get_package_share_directory("my_teleop_joy"), "rviz", "turtlebot_config.rviz"),
        os.path.join(tb4_planner_pkg, "rviz", "planner_config.rviz"),
    ]
    for cfg in possible_rviz:
        if os.path.exists(cfg):
            rviz_config_file = cfg
            break

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time"
        ),
        
        DeclareLaunchArgument(
            "map_yaml_file",
            default_value="/home/evinia/robmob_ws/src/Rob_mob_plannification/tb4_astar_planner/map/test_map1.yaml",
            description="Full path to map YAML file"
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_gazebo_pkg, "launch", "turtlebot3_dqn_stage2.launch.py")
            ),
        ),

        Node(
            package="tb4_astar_planner",
            executable="static_map_publisher",
            name="static_map_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "map_yaml_file": map_yaml_file,
                "publish_rate": 1.0,  
            }]
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_tf",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            parameters=[{"use_sim_time": use_sim_time}],
        ),

        Node(
            package="tb4_astar_planner",
            executable="planner_node_with_saved_map",
            name="tb4_astar_planner",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "map_yaml_path": map_yaml_file,

                "map_topic": "/map",
                "initialpose_topic": "/initialpose",
                "goal_topic": "/goal_pose",
                "path_topic": "/astar_path",
                "cmd_vel_topic": "/cmd_vel",

                "global_frame": "map",
                "base_frame": "base_footprint",

                "robot_radius_m": 0.20,
                "occ_thresh": 50,
                "allow_diag": True,
                "unknown_is_free": False,
                "auto_use_robot_pose": True,

                "lookahead_m": 0.30,
                "v_max": 0.20,
                "w_max": 1.2,
                "yaw_kp": 1.8,
                "goal_tolerance_m": 0.12,
            }],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file] if rviz_config_file else [],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        ),
    ]) 