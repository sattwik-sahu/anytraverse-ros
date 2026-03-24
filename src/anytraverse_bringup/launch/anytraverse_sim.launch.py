import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def generate_launch_description():
    anytraverse_bringup_share_dir = get_package_share_directory("anytraverse_bringup")
    anytraverse_share_dir = get_package_share_directory("anytraverse_ros")
    moonlab_robots_share_dir = get_package_share_directory("moonlab_robots")

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
    unc_map_topic_arg = DeclareLaunchArgument(
        "unc_map_topic",
        default_value="/anytraverse/unc_map",
        description="Topic to remap for uncertainty map",
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
    gated_cmd_vel_topic_arg = DeclareLaunchArgument(
        name="gated_cmd_vel_topic",
        default_value="/anytraverse/cmd_vel",
        description="Topic for AnyTraverse gated cmd_vel output (for HOC)",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Whether to use simulation time (e.g. Gazebo or Webots)",
    )
    nav_params_file_arg = DeclareLaunchArgument(
        name="nav_params_file",
        description="Path to the Nav2 parameters file to use in simulation",
    )

    # Start AnyTraverse node
    anytraverse_params_file_path = PathJoinSubstitution(
        [
            anytraverse_bringup_share_dir,
            "config",
            [LaunchConfiguration("robot"), ".yaml"],
        ]
    )
    anytraverse_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=os.path.join(
                anytraverse_share_dir, "launch", "anytraverse.launch.py"
            ),
        ),
        launch_arguments={
            "params_file": anytraverse_params_file_path,
            "init_prompt": LaunchConfiguration("init_prompt"),
            "rgb_topic": LaunchConfiguration("camera_rgb_topic"),
            "trav_map_topic": LaunchConfiguration("trav_map_topic"),
            "unc_map_topic": LaunchConfiguration("unc_map_topic"),
        }.items(),
    )

    cmd_vel_gating_node = Node(
        package="anytraverse_ros",
        executable="cmd_vel_gating_node",
        name="cmd_vel_gating",
        output="screen",
        remappings={
            "/anytraverse/cmd_vel": LaunchConfiguration("gated_cmd_vel_topic"),
        }.items(),
    )

    # 4. Start the 'obstacle_pcl_node' with remappings
    obstacle_pcl_node = Node(
        package="seg_to_obst",
        executable="seg_to_obst_pcl_node",
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

    # Navigation launch with obstacle topic and params file
    navigation_launch_path = os.path.join(
        moonlab_robots_share_dir, "launch", "navigation.launch.py"
    )
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_path),
        launch_arguments={
            "obstacle_topic": LaunchConfiguration("obstacle_topic"),
            "params_file": LaunchConfiguration("nav_params_file"),
            "use_sim_time": "true",
            "namespace": LaunchConfiguration("robot"),
        }.items(),
    )

    return LaunchDescription(
        [
            obstacle_topic_arg,
            trav_map_topic_arg,
            unc_map_topic_arg,
            camera_rgb_topic_arg,
            camera_rgb_info_topic_arg,
            camera_depth_topic_arg,
            camera_depth_info_topic_arg,
            camera_optical_frame_arg,
            gated_cmd_vel_topic_arg,
            robot_arg,
            init_prompt_arg,
            use_sim_time_arg,
            nav_params_file_arg,
            anytraverse_launch,
            obstacle_pcl_node,
            navigation_launch,
            cmd_vel_gating_node,
        ]
    )
