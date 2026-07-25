from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetRemap
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

    # Boolean arguments with capitalized defaults
    sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo/Webots) clock if True",
    )
    composition_arg = DeclareLaunchArgument(
        "use_composition",
        default_value="False",
        description="Whether to use composed bringup",
    )
    respawn_arg = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn failed nodes",
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
                    "use_composition": LaunchConfiguration("use_composition"),
                    "use_respawn": LaunchConfiguration("use_respawn"),
                    "autostart": "True",
                }.items(),
            ),
        ]
    )

    return LaunchDescription(
        [
            obstacle_topic_arg,
            params_file_arg,
            sim_time_arg,
            composition_arg,
            respawn_arg,
            nav2_group,
        ]
    )
