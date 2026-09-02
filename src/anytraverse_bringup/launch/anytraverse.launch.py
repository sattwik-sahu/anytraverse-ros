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
    imu_topic_arg = DeclareLaunchArgument(
        name="imu_topic",
        default_value="oak/imu/data",
        description="Topic for the IMU data feed",
    )
    camera_frame_id_arg = DeclareLaunchArgument(
        name="camera_frame_id",
        default_value="camera_frame",
        description="TF frame ID for the camera frame",
    )
    gated_cmd_vel_topic_arg = DeclareLaunchArgument(
        name="gated_cmd_vel_topic",
        default_value="/anytraverse/cmd_vel",
        description="Topic for AnyTraverse gated cmd_vel output (for HOC)",
    )

    # Standardized capitalized Boolean arguments to prevent NameError in python evaluations
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation clock if True",
    )
    use_composition_arg = DeclareLaunchArgument(
        "use_composition",
        default_value="False",
        description="Whether to use composed bringup for Nav2",
    )
    use_respawn_arg = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn failed Nav2 nodes",
    )

    # Start the camera
    oakd_launch_path = os.path.join(
        get_package_share_directory("oakd_ros"), "launch", "oak_vio.launch.py"
    )
    oakd_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(oakd_launch_path),
        launch_arguments={
            "camera_rgb_topic": LaunchConfiguration("camera_rgb_topic"),
            "camera_rgb_info_topic": LaunchConfiguration("camera_rgb_info_topic"),
            "camera_depth_topic": LaunchConfiguration("camera_depth_topic"),
            "camera_depth_info_topic": LaunchConfiguration("camera_depth_info_topic"),
            "imu_topic": LaunchConfiguration("imu_topic"),
            "camera_frame_id": LaunchConfiguration("camera_frame_id"),
        }.items(),
    )

    # The robot launch files
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
        remappings=[
            ("/anytraverse/cmd_vel", LaunchConfiguration("gated_cmd_vel_topic")),
        ],
    )

    # Navigation launch (passing sim_time, composition, and respawn arguments)
    navigation_launch_path = os.path.join(
        moonlab_robots_share_dir, "launch", "navigation.launch.py"
    )
    nav_params_file_path = PathJoinSubstitution(
        [
            moonlab_robots_share_dir,
            "config",
            LaunchConfiguration("robot"),
            "nav2_params.yaml",
        ]
    )
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_path),
        launch_arguments={
            "params_file": nav_params_file_path,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "use_composition": LaunchConfiguration("use_composition"),
            "use_respawn": LaunchConfiguration("use_respawn"),
        }.items(),
    )

    return LaunchDescription(
        [
            trav_map_topic_arg,
            unc_map_topic_arg,
            camera_rgb_topic_arg,
            camera_rgb_info_topic_arg,
            camera_depth_topic_arg,
            camera_depth_info_topic_arg,
            imu_topic_arg,
            camera_frame_id_arg,
            gated_cmd_vel_topic_arg,
            robot_arg,
            init_prompt_arg,
            use_sim_time_arg,
            use_composition_arg,
            use_respawn_arg,
            oakd_launch,
            robot_launch,
            anytraverse_launch,
            navigation_launch,
            cmd_vel_gating_node,
        ]
    )
