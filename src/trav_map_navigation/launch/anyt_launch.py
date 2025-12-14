import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch_ros.actions import Node


def generate_launch_description():
    trav_map_navigation_share_dir = get_package_share_directory("trav_map_navigation")

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
            ("/obstacle_points", "/anytraverse/obstacle_points"),
        ],
    )

    # 4. Include the 'navigation_launch.py' file with argument override
    navigation_launch_path = os.path.join(
        trav_map_navigation_share_dir, "launch", "navigation_launch.py"
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_path),
        launch_arguments={"obstacle_topic": "/anytraverse/obstacle_points"}.items(),
    )

    return LaunchDescription(
        [
            oakd_vio_launch,
            anytraverse_node,
            obstacle_pcl_node,
            navigation_launch,
        ]
    )
