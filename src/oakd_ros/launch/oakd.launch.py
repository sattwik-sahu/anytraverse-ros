from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 1. Arguments
    rgb_topic_arg = DeclareLaunchArgument(
        "camera_rgb_topic", default_value="/oakd/rgb/image_raw"
    )
    rgb_info_topic_arg = DeclareLaunchArgument(
        "camera_rgb_info_topic", default_value="/oakd/rgb/camera_info"
    )
    depth_topic_arg = DeclareLaunchArgument(
        "camera_depth_topic", default_value="/oakd/depth/image_raw"
    )

    # 2. Substitutions
    rgb_in = LaunchConfiguration("camera_rgb_topic")
    depth_in = LaunchConfiguration("camera_depth_topic")

    # 3. Driver Node
    oakd_node = Node(
        package="oakd_ros",
        executable="oakd_node",
        name="oakd_node",
        output="screen",
        remappings=[
            ("/oakd/rgb/image_raw", rgb_in),
            ("/oakd/depth/image_raw", depth_in),
            ("/oakd/rgb/camera_info", LaunchConfiguration("camera_rgb_info_topic")),
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

    return LaunchDescription(
        [
            rgb_topic_arg,
            rgb_info_topic_arg,
            depth_topic_arg,
            oakd_node,
            rgb_republisher,
        ]
    )
