import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription


def generate_launch_description():
    trav_map_navigation_share_dir = get_package_share_directory("trav_map_navigation")
    anytraverse_share_dir = get_package_share_directory("anytraverse_ros")
    moonlab_robots_share_dir = get_package_share_directory("moonlab_robots")
    oakd_ros_share_dir = get_package_share_directory("oakd_ros")

    # Args
    obstacle_topic_arg = DeclareLaunchArgument(
        "obstacle_topic",
        default_value="/anytraverse/obstacle_points",
        description="Topic to remap for obstacle avoidance",
    )
    trav_map_topic_arg = DeclareLaunchArgument(
        "trav_map_topic",
        default_value="/anytraverse/trav_map",
        description="Topic to remap for traversability map",
    )
    robot_arg = DeclareLaunchArgument(
        "robot", default_value="default", description="Name of the robot to launch"
    )
    init_prompt_arg = DeclareLaunchArgument(
        name="init_prompt",
        description="Initial prompts for the AnyTraverse pipeline",
    )
    camera_rgb_topic_arg = DeclareLaunchArgument(
        name="camera_rgb_topic",
        default_value="/camera/rgb/image_raw",
        description="Topic for the RGB camera feed",
    )
    camera_rgb_info_topic_arg = DeclareLaunchArgument(
        name="camera_rgb_info_topic",
        default_value="/camera/rgb/camera_info",
        description="Topic for the RGB camera info",
    )
    camera_depth_topic_arg = DeclareLaunchArgument(
        name="camera_depth_topic",
        default_value="/camera/depth/image_raw",
        description="Topic for the depth camera feed",
    )
    camera_depth_info_topic_arg = DeclareLaunchArgument(
        name="camera_depth_info_topic",
        default_value="/camera/depth/camera_info",
        description="Topic for the depth camera info",
    )
    camera_optical_frame_arg = DeclareLaunchArgument(
        name="camera_optical_frame",
        default_value="camera_rgb_optical_frame",
        description="TF frame ID for the camera optical frame",
    )

    # Start the camera and robot
    oakd_node = Node(
        package="oakd_ros",
        executable="oakd_node",
        name="oakd_node",
        output="screen",
    )
    robot_launch_path = os.path.join(
        moonlab_robots_share_dir, "launch", "robot.launch.py"
    )
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_launch_path),
        launch_arguments={"robot": LaunchConfiguration("robot")}.items(),
    )

    # Start AnyTraverse node
    anytraverse_params_file_path = PathJoinSubstitution(
        [
            anytraverse_share_dir,
            "config",
            ["params_", LaunchConfiguration("robot"), ".yaml"],
        ]
    )
    anytraverse_node = Node(
        package="anytraverse_ros",
        executable="anytraverse_node",
        name="anytraverse_node",
        output="screen",
        parameters=[
            anytraverse_params_file_path,
            {
                "init_prompt": ParameterValue(
                    LaunchConfiguration("init_prompt"), value_type=str
                )
            },
        ],
        remappings=[
            ("/camera/rgb/image_raw", LaunchConfiguration("camera_rgb_topic")),
        ],
    )

    cmd_vel_gating_node = Node(
        package="anytraverse_ros",
        executable="cmd_vel_gating_node",
        name="cmd_vel_gating",
        output="screen",
    )

    # 4. Start the 'obstacle_pcl_node' with remappings
    obstacle_pcl_node = Node(
        package="trav_map_navigation",
        executable="obstacle_pcl_node",
        name="obstacle_pcl",
        output="screen",
        remappings=[
            ("/trav_map", LaunchConfiguration("trav_map_topic")),
            ("/obstacle_points", LaunchConfiguration("obstacle_topic")),
            ("/camera/depth/image_raw", LaunchConfiguration("camera_depth_topic")),
            (
                "/camera/depth/camera_info",
                LaunchConfiguration("camera_depth_info_topic"),
            ),
        ],
        parameters=[
            {
                "camera_optical_frame_id": LaunchConfiguration("camera_optical_frame"),
            }
        ],
    )

    # 4. Include the 'navigation_launch.py' file
    navigation_launch_path = os.path.join(
        trav_map_navigation_share_dir, "launch", "navigation_launch.py"
    )
    nav_params_file_path = PathJoinSubstitution(
        [
            trav_map_navigation_share_dir,
            "config",
            ["params_", LaunchConfiguration("robot"), ".yaml"],
        ]
    )
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_path),
        launch_arguments={
            "obstacle_topic": LaunchConfiguration("obstacle_topic"),
            "params_file": nav_params_file_path,
        }.items(),
    )

    return LaunchDescription(
        [
            obstacle_topic_arg,
            trav_map_topic_arg,
            camera_rgb_topic_arg,
            camera_rgb_info_topic_arg,
            camera_depth_topic_arg,
            camera_depth_info_topic_arg,
            camera_optical_frame_arg,
            robot_arg,
            init_prompt_arg,
            oakd_node,
            robot_launch,
            anytraverse_node,
            obstacle_pcl_node,
            navigation_launch,
            cmd_vel_gating_node,
        ]
    )
