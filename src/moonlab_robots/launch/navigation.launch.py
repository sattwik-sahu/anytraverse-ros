import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav2_bringup_pkg = FindPackageShare("nav2_bringup")

    # Obstacle pointclouds topic arg
    obstacle_topic_arg = DeclareLaunchArgument(
        "obstacle_topic",
        default_value="/obstacle_points",
        description="Topic to remap for obstacle avoidance",
    )

    # Nav2 params file arg
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        description="Path to nav2 params file",
    )

    # Added this arg for easy switching to Webots later
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo/Webots) clock if True",
    )

    # Namespace arg
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Robot namespace",
    )

    nav2_group = GroupAction(
        actions=[
            PushRosNamespace(LaunchConfiguration("namespace")),
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
                    "autostart": "true",
                }.items(),
            ),
        ]
    )

    return LaunchDescription(
        [
            obstacle_topic_arg,
            params_file_arg,
            use_sim_time_arg,
            namespace_arg,
            nav2_group,
        ]
    )
