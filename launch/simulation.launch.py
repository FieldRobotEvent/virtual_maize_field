from __future__ import annotations

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    paused = LaunchConfiguration("paused")
    headless = LaunchConfiguration("headless")
    world_path = LaunchConfiguration("world_path")
    world_name = LaunchConfiguration("world_name")
    check_for_updates = LaunchConfiguration("check_for_updates")

    # Create launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time", default_value="True"
    )
    declare_gui_cmd = DeclareLaunchArgument(name="gui", default_value="True")
    declare_paused_cmd = DeclareLaunchArgument(name="paused", default_value="False")
    declare_headless_cmd = DeclareLaunchArgument(name="headless", default_value="False")
    declare_world_path_cmd = DeclareLaunchArgument(
        name="world_path",
        default_value=get_package_share_directory("virtual_maize_field") + "/worlds/",
    )
    declare_world_name_cmd = DeclareLaunchArgument(
        name="world_name", default_value="generated.world"
    )
    declare_check_for_updates_cmd = DeclareLaunchArgument(
        name="check_for_updates", default_value="True"
    )

    # Create nodes
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("gazebo_ros") + "/launch/gzserver.launch.py"
        ),
        launch_arguments={
            "world": [world_path, world_name],
            "pause": paused,
            "verbose": "true",
            "use_sim_time": use_sim_time,
        }.items(),
    )

    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("gazebo_ros") + "/launch/gzclient.launch.py"
        ),
        condition=IfCondition(PythonExpression([gui, " and not ", headless])),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    start_check_updates_node_cmd = Node(
        package="virtual_maize_field",
        executable="check_for_updates",
        name="check_virtual_maize_field_updates",
        condition=IfCondition(check_for_updates),
        arguments={"use_sim_time": use_sim_time}.items(),
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_gui_cmd)
    ld.add_action(declare_paused_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_world_path_cmd)
    ld.add_action(declare_world_name_cmd)
    ld.add_action(declare_check_for_updates_cmd)

    # Add nodes
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_check_updates_node_cmd)

    return ld
