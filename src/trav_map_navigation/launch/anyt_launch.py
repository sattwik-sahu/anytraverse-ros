import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def generate_launch_description():
    trav_map_navigation_share_dir = get_package_share_directory("trav_map_navigation")

    # Args
    obstacle_topic_arg = DeclareLaunchArgument(
        "obstacle_topic",
        default_value="/anytraverse/obstacle_points",
        description="Topic to remap for obstacle avoidance",
    )
    robot_name_arg = DeclareLaunchArgument(
        "robot_name", default_value="default", description="Name of the robot to launch"
    )

    # 1. Include the OAK-D VIO launch file
    oakd_ros_share_dir = get_package_share_directory("oakd_ros")
    oakd_vio_launch_path = os.path.join(
        oakd_ros_share_dir, "launch", "oakd_vio_launch.yaml"
    )
    oakd_vio_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(oakd_vio_launch_path)
    )

    # 2. Start the 'anytraverse_node'
    anytraverse_node = Node(
        package="anytraverse_ros",
        executable="anytraverse_node",
        name="anytraverse",
        output="screen",
    )

    # 3. Start the 'obstacle_pcl_node' with remappings
    obstacle_pcl_node = Node(
        package="trav_map_navigation",
        executable="obstacle_pcl_node",
        name="obstacle_pcl",
        output="screen",
        remappings=[
            ("/trav_map", "/anytraverse/trav_map"),
            ("/obstacle_points", LaunchConfiguration("obstacle_topic")),
        ],
    )

    # 4. Include the 'navigation_launch.py' file
    navigation_launch_path = os.path.join(
        trav_map_navigation_share_dir, "launch", "navigation_launch.py"
    )

    params_file_path = PathJoinSubstitution(
        [
            trav_map_navigation_share_dir,
            "config",  # Adjust this if your params are not in a 'config' subfolder
            ["params_", LaunchConfiguration("robot_name"), ".yaml"],
        ]
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_path),
        launch_arguments={
            "obstacle_topic": LaunchConfiguration("obstacle_topic"),
            "params_file": params_file_path,
        }.items(),
    )

    return LaunchDescription(
        [
            obstacle_topic_arg,
            robot_name_arg,
            oakd_vio_launch,
            anytraverse_node,
            obstacle_pcl_node,
            navigation_launch,
        ]
    )
