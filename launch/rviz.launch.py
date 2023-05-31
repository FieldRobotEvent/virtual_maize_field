from __future__ import annotations

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    # Create nodes
    start_rviz_cmd = Node(
        package="rviz",
        executable="rviz",
        name="rviz",
        arguments={
            "d": get_package_share_directory("virtual_maize_field")
            + "/rviz/config.rviz"
        }.items(),
    )

    ld = LaunchDescription()

    # Add nodes
    ld.add_action(start_rviz_cmd)

    return ld
