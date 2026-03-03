import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot",
                default_value="kombai",
                description="Name of the robot (key in the YAML config)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )


def launch_setup(context, *args, **kwargs):
    # Resolve the configurations
    robot_name = LaunchConfiguration("robot").perform(context)
    pkg_path = get_package_share_directory("moonlab_robots")

    urdf_file = os.path.join(pkg_path, "urdf", "robot.urdf.xacro")
    config_file = os.path.join(pkg_path, "config", "robot_camera_mounts.yaml")

    # Pass the file path and robot name as Xacro arguments
    robot_description_content = Command(
        [
            "xacro ",
            urdf_file,
            " config_path:=",
            config_file,
            " robot_name:=",
            robot_name,
        ]
    )

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description_content}],
        )
    ]
