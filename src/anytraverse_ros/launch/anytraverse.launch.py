from launch import LaunchDescription as LaunchDescription
from launch_ros.actions import Node as Node
from launch.actions import DeclareLaunchArgument as DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue as ParameterValue


def generate_launch_description():
    # Declare launch arguments
    rgb_topic_arg = DeclareLaunchArgument(
        name="rgb_topic",
        default_value="/camera/rgb/image_raw",
        description="Topic for the RGB camera feed",
    )
    trav_map_topic_arg = DeclareLaunchArgument(
        name="trav_map_topic",
        default_value="/anytraverse/trav_map",
        description="Topic for the traversability map output",
    )
    unc_map_topic_arg = DeclareLaunchArgument(
        name="unc_map_topic",
        default_value="/anytraverse/unc_map",
        description="Topic for the uncertainty map output",
    )
    params_file_arg = DeclareLaunchArgument(
        name="params_file",
        description="Path to the YAML file containing parameters for the AnyTraverse node",
    )
    init_prompt_arg = DeclareLaunchArgument(
        name="init_prompt",
        description="Initial prompts for the AnyTraverse pipeline",
    )

    # Create the AnyTraverse node
    anytraverse_node = Node(
        package="anytraverse_ros",
        executable="anytraverse_node",
        name="anytraverse_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "init_prompt": ParameterValue(
                    LaunchConfiguration("init_prompt"), value_type=str
                ),
            },
        ],
        remappings={
            "/camera/rgb/image_raw": LaunchConfiguration("rgb_topic"),
            "/anytraverse/trav_map": LaunchConfiguration("trav_map_topic"),
            "/anytraverse/unc_map": LaunchConfiguration("unc_map_topic"),
        }.items(),
    )

    # Create republishers for the traversability and uncertainty maps
    trav_map_republisher = Node(
        package="image_transport",
        executable="republish",
        name="trav_map_republisher",
        parameters=[
            {
                "in_transport": "raw",
                "out_transport": "compressed",
            }
        ],
        remappings=[
            ("in", LaunchConfiguration("trav_map_topic")),
            ("out/compressed", [LaunchConfiguration("trav_map_topic"), "/compressed"]),
        ],
        output="screen",
    )
    unc_map_republisher = Node(
        package="image_transport",
        executable="republish",
        name="unc_map_republisher",
        parameters=[
            {
                "in_transport": "raw",
                "out_transport": "compressed",
            }
        ],
        remappings=[
            ("in", LaunchConfiguration("unc_map_topic")),
            ("out/compressed", [LaunchConfiguration("unc_map_topic"), "/compressed"]),
        ],
        output="screen",
    )

    return LaunchDescription(
        [rgb_topic_arg, anytraverse_node, trav_map_republisher, unc_map_republisher]
    )
