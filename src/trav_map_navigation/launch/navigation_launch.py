import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    trav_map_nav_pkg = FindPackageShare("trav_map_navigation")
    nav2_bringup_pkg = FindPackageShare("nav2_bringup")

    default_params_file = PathJoinSubstitution(
        [trav_map_nav_pkg, "config", "nav", "params.yaml"]
    )

    # ARGS
    obstacle_topic_arg = DeclareLaunchArgument(
        "obstacle_topic",
        default_value="/obstacle_points",
        description="Topic to remap for obstacle avoidance",
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Path to nav2 params file",
    )

    # Added this arg for easy switching to Webots later
    sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo/Webots) clock if True",
    )

    # NODES
    # "Inverted" Static TF: odom -> camera_link (from VIO) -> base_link (static)
    tf_fix_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_to_base",
        arguments=["-0.2", "0", "-1.0", "0", "0", "0", "camera_link", "base_link"],
        output="screen",
    )

    nav2_group = GroupAction(
        actions=[
            SetRemap(src="/obstacle_points", dst=LaunchConfiguration("obstacle_topic")),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [nav2_bringup_pkg, "launch", "navigation_launch.py"]
                    )
                ),
                launch_arguments={
                    "params_file": LaunchConfiguration("params_file"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "autostart": "True",
                    # "start_docking_server": "False",
                }.items(),
            ),
        ]
    )

    return LaunchDescription(
        [obstacle_topic_arg, params_file_arg, sim_time_arg, tf_fix_node, nav2_group]
    )
