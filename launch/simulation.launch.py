from __future__ import annotations

from os import environ, path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description() -> LaunchDescription:
    _ros_home_path = environ.get("ROS_HOME", path.join(path.expanduser("~"), ".ros"))
    cache_dir = path.join(_ros_home_path, "virtual_maize_field/")

    use_sim_time = LaunchConfiguration("use_sim_time", default=True)
    gui = LaunchConfiguration("gui", default=True)
    paused = LaunchConfiguration("paused", default=False)
    headless = LaunchConfiguration("headless", default=False)
    world_path = LaunchConfiguration("world_path", default=cache_dir)
    world_name = LaunchConfiguration("world_name", default="generated.world")

    gz_server_launch_file = path.join(
        get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py"
    )
    gz_client_launch_file = path.join(
        get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py"
    )

    gz_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_server_launch_file]),
        launch_arguments={
            "world": [world_path, world_name],
            "pause": paused,
            "verbose": "true",
            "use_sim_time": use_sim_time,
        }.items(),
    )

    gz_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_client_launch_file]),
        condition=IfCondition(PythonExpression([gui, " and not ", headless])),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(name="use_sim_time", default_value=use_sim_time),
            DeclareLaunchArgument(name="gui", default_value=gui),
            DeclareLaunchArgument(name="paused", default_value=paused),
            DeclareLaunchArgument(name="headless", default_value=headless),
            DeclareLaunchArgument(name="world_path", default_value=world_path),
            DeclareLaunchArgument(name="world_name", default_value=world_name),
            gz_server_launch,
            gz_client_launch,
        ]
    )
