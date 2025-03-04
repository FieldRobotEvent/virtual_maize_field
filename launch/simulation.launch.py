from __future__ import annotations

from os import environ, path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, SomeSubstitutionsType
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)

from launch_ros.actions import Node


def construct_gz_args(
    world_file: SomeSubstitutionsType,
    paused: SomeSubstitutionsType,
    headless: SomeSubstitutionsType,
    verbose: SomeSubstitutionsType,
) -> SomeSubstitutionsType:
    paused_arg = PythonExpression(['"" if "', paused, '".lower() == "true" else "-r "'])
    headless_arg = PythonExpression(
        ['"-s " if "', headless, '".lower() == "true" else ""']
    )
    verbose_arg = PythonExpression(
        ['"-v4 " if "', verbose, '".lower() == "true" else ""']
    )

    return PythonExpression(
        ["'", headless_arg, verbose_arg, paused_arg, world_file, "'"]
    )


def generate_launch_description() -> LaunchDescription:
    _ros_home_path = environ.get("ROS_HOME", path.join(path.expanduser("~"), ".ros"))
    cache_dir = path.join(_ros_home_path, "virtual_maize_field/")

    use_sim_time = LaunchConfiguration("use_sim_time")
    paused = LaunchConfiguration("paused")
    headless = LaunchConfiguration("headless")
    verbose = LaunchConfiguration("verbose")
    world_path = LaunchConfiguration("world_path")
    world_name = LaunchConfiguration("world_name")

    # Create launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time", default_value="True"
    )
    declare_paused_cmd = DeclareLaunchArgument(
        name="paused", default_value="False", description="Start simulation paused."
    )
    declare_headless_cmd = DeclareLaunchArgument(
        name="headless",
        default_value="False",
        description="Start headless simulation (without rendering).",
    )
    declare_verbose_cmd = DeclareLaunchArgument(
        name="verbose",
        default_value="False",
        description="Increase verbosity of Gazebo",
    )
    declare_world_path_cmd = DeclareLaunchArgument(
        name="world_path",
        default_value=cache_dir,
        description="Path to the directory containing the world SDF files.",
    )
    declare_world_name_cmd = DeclareLaunchArgument(
        name="world_name",
        default_value="generated.world",
        description="Name of the world file.",
    )

    gz_args = construct_gz_args(
        PathJoinSubstitution([world_path, world_name]),
        paused,
        headless,
        verbose,
    )

    environment = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        path.join(get_package_share_directory("virtual_maize_field"), "models"),
    )

    log_gz_args = LogInfo(msg=["Start Ignition Gazebo with gz_args: '", gz_args, "'"])

    # Create nodes
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            ),
        ),
        launch_arguments={
            "gz_args": gz_args,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    sim_time_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        condition=IfCondition(use_sim_time),
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_paused_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_verbose_cmd)
    ld.add_action(declare_world_path_cmd)
    ld.add_action(declare_world_name_cmd)

    # Append models to environment
    ld.add_action(environment)

    # Add nodes
    ld.add_action(log_gz_args)
    ld.add_action(gz)
    ld.add_action(sim_time_bridge)

    return ld
