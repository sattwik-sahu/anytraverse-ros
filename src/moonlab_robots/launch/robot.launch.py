import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="robot"),
            DeclareLaunchArgument("base_frame_id", default_value="base_link"),
            DeclareLaunchArgument("camera_frame_id", default_value="camera_frame"),
            OpaqueFunction(function=launch_setup),
        ]
    )


def launch_setup(context, *args, **kwargs) -> list[Node]:
    robot = LaunchConfiguration("robot").perform(context=context)
    base_frame = LaunchConfiguration("base_frame_id").perform(context=context)
    camera_frame = LaunchConfiguration("camera_frame_id").perform(context=context)

    pkg_path = get_package_share_directory("moonlab_robots")
    config_file = os.path.join(pkg_path, "config", robot, "camera_mount.yaml")

    with open(config_file, "r") as f:
        config = yaml.safe_load(f)

    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_static_tf",
            arguments=[
                str(config["x"]),
                str(config["y"]),
                str(config["z"]),
                str(config["roll"]),
                str(config["pitch"]),
                str(config["yaw"]),
                base_frame,
                camera_frame,
            ],
        )
    ]
