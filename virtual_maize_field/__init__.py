from __future__ import annotations

from os import environ
from pathlib import Path

__all__ = ["get_spawner_launch_file", "get_driving_pattern_file", "get_markers_file"]


class NoWorldGeneratedException(FileNotFoundError):
    pass


_cache_dir = environ.get("ROS_HOME", str(Path.home() / ".ros"))


def get_spawner_launch_file() -> str:
    world_path = Path(_cache_dir) / "virtual_maize_field" / "robot_spawner.launch.py"

    if not world_path.is_file():
        raise NoWorldGeneratedException(
            "No robot spawner launch file is generated! First generate launch file by"
            " 'ros2 run virtual_maize_field generate_world fre22_task_navigation_mini'"
            " (for example)."
        )

    return str(world_path)


def get_driving_pattern_file() -> str:
    driving_pattern_path = (
        Path(_cache_dir) / "virtual_maize_field" / "driving_pattern.txt"
    )

    if not driving_pattern_path.is_file():
        raise NoWorldGeneratedException(
            "No driving pattern file is generated! First generate driving pattern by"
            " 'ros2 run virtual_maize_field generate_world fre22_task_navigation_mini'"
            " (for example)."
        )

    return str(driving_pattern_path)


def get_markers_file() -> str:
    markers_path = Path(_cache_dir) / "virtual_maize_field" / "markers.csv"

    if not markers_path.is_file():
        raise NoWorldGeneratedException(
            "No markers file is generated! First generate marker file by"
            " 'ros2 run virtual_maize_field generate_world  fre21_task_3'"
            " (for example)."
        )

    return str(markers_path)
