import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 1. Arguments
    rgb_topic_arg = DeclareLaunchArgument(
        "camera_rgb_topic", default_value="/oak/camera/image_raw"
    )
    rgb_info_topic_arg = DeclareLaunchArgument(
        "camera_rgb_info_topic", default_value="/oak/camera/camera_info"
    )
    depth_topic_arg = DeclareLaunchArgument(
        "camera_depth_topic", default_value="/oak/depth/image_raw"
    )
    depth_info_topic_arg = DeclareLaunchArgument(
        "camera_depth_info_topic", default_value="/oak/depth/camera_info"
    )
    imu_topic_arg = DeclareLaunchArgument("imu_topic", default_value="oak/imu/data")
    camera_frame_id_arg = DeclareLaunchArgument(
        "camera_frame_id", default_value="camera_frame"
    )

    # 2. Substitutions
    rgb_in = LaunchConfiguration("camera_rgb_topic")
    depth_in = LaunchConfiguration("camera_depth_topic")

    # 3. Driver Node
    oakd_node = Node(
        package="oakd_ros",
        executable="oak_vio_node",
        name="oak_vio_node",
        output="screen",
        remappings=[
            ("/oak/rgb/image_raw", rgb_in),
            ("/oak/depth/image_raw", depth_in),
            ("/oak/rgb/camera_info", LaunchConfiguration("camera_rgb_info_topic")),
            ("/oak/depth/camera_info", LaunchConfiguration("camera_depth_info_topic")),
            ("/oak/imu/data", LaunchConfiguration("imu_topic")),
        ],
    )

    # 4. RGB Republisher
    rgb_republisher = Node(
        package="image_transport",
        executable="republish",
        name="rgb_republisher",
        # Fixes the "out_transport not recognized" error
        parameters=[
            {
                "in_transport": "raw",
                "out_transport": "compressed",
            }
        ],
        # Fixes the "/out/compressed" naming error
        # Key must be just "in" and "out" (no slashes, no suffixes)
        remappings=[
            ("/in", rgb_in),
            ("/out/compressed", [rgb_in, "/compressed"]),
        ],
        output="screen",
    )

    # Launching the OAK camera description
    depthai_descriptions_launch = os.path.join(
        get_package_share_directory("depthai_descriptions"), "launch", "urdf_launch.py"
    )
    camera_frame = LaunchConfiguration("camera_frame_id")
    depthai_urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(depthai_descriptions_launch),
        launch_arguments={"parent_frame": camera_frame}.items(),
    )

    return LaunchDescription(
        [
            rgb_topic_arg,
            rgb_info_topic_arg,
            depth_topic_arg,
            depth_info_topic_arg,
            imu_topic_arg,
            camera_frame_id_arg,
            oakd_node,
            rgb_republisher,
            depthai_urdf_launch,
        ]
    )
