from __future__ import annotations

from pathlib import Path
from subprocess import run

import pytest
from ament_index_python import get_package_share_directory

CONFIG_PATH = Path(get_package_share_directory("virtual_maize_field")) / "config"
CONFIG_FILES = [(file.stem) for file in CONFIG_PATH.glob("*.yaml")]


@pytest.mark.parametrize("config_file", CONFIG_FILES)
def test_predefinded_worlds(config_file: str) -> None:
    # The fre22_task_mapping tasks cannot be tested because the dandelion models are not
    # available on Github. So, skip these tests.
    if "fre22_task_mapping" in config_file:
        pytest.skip()

    # TODO: remove once https://github.com/FieldRobotEvent/virtual_maize_field/issues/45 is fixed
    if not "_mini" in config_file:
        pytest.skip()

    process = run(["ros2", "run", "virtual_maize_field", "generate_world", config_file])
    assert process.returncode == 0, f"Cannot create {config_file}!"
