from __future__ import annotations

from os import environ
from pathlib import Path

__all__ = ["get_driving_pattern"]


class NoWorldGeneratedException(FileNotFoundError):
    pass


def get_driving_pattern() -> str:
    cache_dir = environ.get("ROS_HOME", str(Path.home() / ".ros"))
    driving_pattern_path = (
        Path(cache_dir) / "virtual_maize_field" / "driving_pattern.txt"
    )

    if not driving_pattern_path.is_file():
        raise NoWorldGeneratedException(
            "No driving pattern file is generated! First generate driving pattern by"
            " 'rosrun virtual_maize_field generate_world.py fre22_task_navigation_mini'"
            " (for example)."
        )

    return str(driving_pattern_path)
