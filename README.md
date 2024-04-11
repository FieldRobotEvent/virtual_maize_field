# Virtual Maize Field

<p float="left" align="middle">
  <img src="misc/FRE-logo.png" width="250">
</p>
<p float="left" align="middle"> 
  <img src="misc/WUR-logo.png" width="250" style="margin: 10px;"> 
  <img src="https://www.uni-hohenheim.de/typo3conf/ext/uni_layout/Resources/Public/Images/uni-logo-en.svg" width="250" style="margin: 10px;">
  <img src="https://kamaro-engineering.de/wp-content/uploads/2015/03/Kamaro_Logo-1.png" width="250" style="margin: 10px;">
</p>
<p align="middle">
  <a href="https://github.com/psf/black"><img src="https://img.shields.io/badge/code%20style-black-000000.svg" alt="Code style: black"/></a>
   <a href="https://pycqa.github.io/isort/"><img src="https://img.shields.io/badge/%20imports-isort-%231674b1?style=flat&labelColor=ef8336"/></a>
  <a href="https://github.com/FieldRobotEvent/Virtual_Field_Robot_Event/discussions"><img src="https://img.shields.io/badge/contributions-welcome-brightgreen.svg?style=flat" alt="contributions welcome"/></a>
  <a href="https://www.gnu.org/licenses/gpl-3.0"><img src="https://img.shields.io/badge/License-GPLv3-blue.svg" alt="License: GPL v3"/></a>
</p>
<p align="middle">
  This is a package to procedurally generate randomized fields with rows of plants for Gazebo.
</p>

![Screenshot of a generated map with maize plants](./misc/screenshot_v5.0_classic.png)

ROS Distro | Gazebo version | Branch | Build status
:---------: | :----: | :----: | :----------: 
**Rolling** | **Gazebo** | [`ros2-gz`](https://github.com/FieldRobotEvent/virtual_maize_field/tree/ros2-gz) | [![ros-rolling-gz](https://github.com/FieldRobotEvent/virtual_maize_field/actions/workflows/ros-rolling-gz-test.yaml/badge.svg?branch=ros2-gz)](https://github.com/FieldRobotEvent/virtual_maize_field/actions/workflows/ros-rolling-gz-test.yaml)
**Rolling** | **Gazebo Classic** | [`ros2`](https://github.com/FieldRobotEvent/virtual_maize_field/tree/ros2) | [![ros-rolling](https://github.com/FieldRobotEvent/virtual_maize_field/actions/workflows/ros-rolling-test.yaml/badge.svg?branch=ros2)](https://github.com/FieldRobotEvent/virtual_maize_field/actions/workflows/ros-rolling-test.yaml)
**Iron** | **Gazebo** | [`ros2-gz`](https://github.com/FieldRobotEvent/virtual_maize_field/tree/ros2-gz) | [![ros-iron-gz](https://github.com/FieldRobotEvent/virtual_maize_field/actions/workflows/ros-iron-gz-test.yaml/badge.svg?branch=ros2-gz)](https://github.com/FieldRobotEvent/virtual_maize_field/actions/workflows/ros-iron-gz-test.yaml)
**Iron** | **Gazebo Classic** | [`ros2`](https://github.com/FieldRobotEvent/virtual_maize_field/tree/ros2) | [![ros-iron](https://github.com/FieldRobotEvent/virtual_maize_field/actions/workflows/ros-iron-test.yaml/badge.svg?branch=ros2)](https://github.com/FieldRobotEvent/virtual_maize_field/actions/workflows/ros-iron-test.yaml)
**Humble** | **Ignition Gazebo** | [`ros2-ign`](https://github.com/FieldRobotEvent/virtual_maize_field/tree/ros2-ign) | [![ros-humble-ign](https://github.com/FieldRobotEvent/virtual_maize_field/actions/workflows/ros-humble-ign-test.yaml/badge.svg?branch=ros2-ign)](https://github.com/FieldRobotEvent/virtual_maize_field/actions/workflows/ros-humble-ign-test.yaml)
**Humble** | **Gazebo Classic** | [`ros2`](https://github.com/FieldRobotEvent/virtual_maize_field/tree/ros2) | [![ros-humble](https://github.com/FieldRobotEvent/virtual_maize_field/actions/workflows/ros-humble-test.yaml/badge.svg?branch=ros2)](https://github.com/FieldRobotEvent/virtual_maize_field/actions/workflows/ros-humble-test.yaml)
**Noetic** | **Gazebo Classic** | [`main`](https://github.com/FieldRobotEvent/virtual_maize_field/tree/main) | [![ros-noetic](https://github.com/FieldRobotEvent/virtual_maize_field/actions/workflows/ros-noetic-test.yaml/badge.svg?branch=main)](https://github.com/FieldRobotEvent/virtual_maize_field/actions/workflows/ros-noetic-test.yaml?branch=main)


## Installation
Clone this repository and build the workspace. Additional you'll need the following packages:

```commandline
rosdep install virtual_maize_field
```

This package has been tested on ROS 2 Humble, Iron and Rolling.

## Generating new maize field worlds
This package includes a script (`virtual_maize_field/generate_world.py`) that can generate randomized agricultural worlds. All parameters are optional and have default values. All comma separated arguments can be scaler as well. 

You can call the script using
```bash
ros2 run virtual_maize_field generate_world
```
The resulting file will be placed in `$ROS_HOME/virtual_maize_field/generated.world`. 

You can use this script by one of the defined config files or specifying the parameters below:
<details>
  <summary>Click to show all possible arguments</summary>
  
  ```
  usage: generate_world [-h] [--row_length ROW_LENGTH]
                      [--rows_curve_budget ROWS_CURVE_BUDGET]
                      [--row_width ROW_WIDTH] [--rows_count ROWS_COUNT]
                      [--row_segments [ROW_SEGMENTS [ROW_SEGMENTS ...]]]
                      [--row_segment_straight_length_min ROW_SEGMENT_STRAIGHT_LENGTH_MIN]
                      [--row_segment_straight_length_max ROW_SEGMENT_STRAIGHT_LENGTH_MAX]
                      [--row_segment_sincurved_offset_min ROW_SEGMENT_SINCURVED_OFFSET_MIN]
                      [--row_segment_sincurved_offset_max ROW_SEGMENT_SINCURVED_OFFSET_MAX]
                      [--row_segment_sincurved_length_min ROW_SEGMENT_SINCURVED_LENGTH_MIN]
                      [--row_segment_sincurved_length_max ROW_SEGMENT_SINCURVED_LENGTH_MAX]
                      [--row_segment_curved_radius_min ROW_SEGMENT_CURVED_RADIUS_MIN]
                      [--row_segment_curved_radius_max ROW_SEGMENT_CURVED_RADIUS_MAX]
                      [--row_segment_curved_arc_measure_min ROW_SEGMENT_CURVED_ARC_MEASURE_MIN]
                      [--row_segment_curved_arc_measure_max ROW_SEGMENT_CURVED_ARC_MEASURE_MAX]
                      [--row_segment_island_radius_min ROW_SEGMENT_ISLAND_RADIUS_MIN]
                      [--row_segment_island_radius_max ROW_SEGMENT_ISLAND_RADIUS_MAX]
                      [--ground_resolution GROUND_RESOLUTION]
                      [--ground_elevation_max GROUND_ELEVATION_MAX]
                      [--ground_headland GROUND_HEADLAND]
                      [--ground_ditch_depth GROUND_DITCH_DEPTH]
                      [--plant_spacing_min PLANT_SPACING_MIN]
                      [--plant_spacing_max PLANT_SPACING_MAX]
                      [--plant_height_min PLANT_HEIGHT_MIN]
                      [--plant_height_max PLANT_HEIGHT_MAX]
                      [--plant_radius PLANT_RADIUS]
                      [--plant_radius_noise PLANT_RADIUS_NOISE]
                      [--plant_placement_error_max PLANT_PLACEMENT_ERROR_MAX]
                      [--plant_mass PLANT_MASS]
                      [--hole_prob [HOLE_PROB [HOLE_PROB ...]]]
                      [--hole_size_max [HOLE_SIZE_MAX [HOLE_SIZE_MAX ...]]]
                      [--crop_types [CROP_TYPES [CROP_TYPES ...]]]
                      [--litters LITTERS]
                      [--litter_types [LITTER_TYPES [LITTER_TYPES ...]]]
                      [--weeds WEEDS]
                      [--weed_types [WEED_TYPES [WEED_TYPES ...]]]
                      [--ghost_objects GHOST_OBJECTS]
                      [--location_markers LOCATION_MARKERS]
                      [--load_from_file LOAD_FROM_FILE] [--seed SEED]
                      [--show_map]
                      [{fre21_task_2_mini,fre21_task_2_fast,fre21_task_1_mini,fre22_task_mapping_mini,fre22_task_navigation,fre21_task_3_fast,fre22_task_mapping,fre21_task_4_fast,fre21_task_2,fre21_task_3_mini,fre22_task_mapping_fast,fre21_task_1,fre21_task_3,fre22_task_navigation_fast,fre21_task_1_fast,fre21_task_4,fre22_task_navigation_mini,fre21_task_4_mini}]

Generate a virtual maize field world for Gazebo.

positional arguments:
  {fre21_task_2_mini,fre21_task_2_fast,fre21_task_1_mini,fre22_task_mapping_mini,fre22_task_navigation,fre21_task_3_fast,fre22_task_mapping,fre21_task_4_fast,fre21_task_2,fre21_task_3_mini,fre22_task_mapping_fast,fre21_task_1,fre21_task_3,fre22_task_navigation_fast,fre21_task_1_fast,fre21_task_4,fre22_task_navigation_mini,fre21_task_4_mini}
                        Config file name in the config folder

optional arguments:
  -h, --help            show this help message and exit
  --row_length ROW_LENGTH
                        default_value: 12.0
  --rows_curve_budget ROWS_CURVE_BUDGET
                        default_value: 1.5707963267948966
  --row_width ROW_WIDTH
                        default_value: 0.75
  --rows_count ROWS_COUNT
                        default_value: 6
  --row_segments [ROW_SEGMENTS [ROW_SEGMENTS ...]]
                        default_value: ['straight', 'curved']
  --row_segment_straight_length_min ROW_SEGMENT_STRAIGHT_LENGTH_MIN
                        default_value: 0.5
  --row_segment_straight_length_max ROW_SEGMENT_STRAIGHT_LENGTH_MAX
                        default_value: 1
  --row_segment_sincurved_offset_min ROW_SEGMENT_SINCURVED_OFFSET_MIN
                        default_value: 0.5
  --row_segment_sincurved_offset_max ROW_SEGMENT_SINCURVED_OFFSET_MAX
                        default_value: 1.5
  --row_segment_sincurved_length_min ROW_SEGMENT_SINCURVED_LENGTH_MIN
                        default_value: 3
  --row_segment_sincurved_length_max ROW_SEGMENT_SINCURVED_LENGTH_MAX
                        default_value: 5
  --row_segment_curved_radius_min ROW_SEGMENT_CURVED_RADIUS_MIN
                        default_value: 3.0
  --row_segment_curved_radius_max ROW_SEGMENT_CURVED_RADIUS_MAX
                        default_value: 10.0
  --row_segment_curved_arc_measure_min ROW_SEGMENT_CURVED_ARC_MEASURE_MIN
                        default_value: 0.3
  --row_segment_curved_arc_measure_max ROW_SEGMENT_CURVED_ARC_MEASURE_MAX
                        default_value: 1.0
  --row_segment_island_radius_min ROW_SEGMENT_ISLAND_RADIUS_MIN
                        default_value: 1.0
  --row_segment_island_radius_max ROW_SEGMENT_ISLAND_RADIUS_MAX
                        default_value: 3.0
  --ground_resolution GROUND_RESOLUTION
                        default_value: 0.02
  --ground_elevation_max GROUND_ELEVATION_MAX
                        default_value: 0.2
  --ground_headland GROUND_HEADLAND
                        default_value: 2.0
  --ground_ditch_depth GROUND_DITCH_DEPTH
                        default_value: 0.3
  --plant_spacing_min PLANT_SPACING_MIN
                        default_value: 0.13
  --plant_spacing_max PLANT_SPACING_MAX
                        default_value: 0.19
  --plant_height_min PLANT_HEIGHT_MIN
                        default_value: 0.3
  --plant_height_max PLANT_HEIGHT_MAX
                        default_value: 0.6
  --plant_radius PLANT_RADIUS
                        default_value: 0.3
  --plant_radius_noise PLANT_RADIUS_NOISE
                        default_value: 0.05
  --plant_placement_error_max PLANT_PLACEMENT_ERROR_MAX
                        default_value: 0.02
  --plant_mass PLANT_MASS
                        default_value: 0.3
  --hole_prob [HOLE_PROB [HOLE_PROB ...]]
                        default_value: [0.06, 0.06, 0.04, 0.04, 0.0, 0.0]
  --hole_size_max [HOLE_SIZE_MAX [HOLE_SIZE_MAX ...]]
                        default_value: [7, 5, 5, 3, 0, 0]
  --crop_types [CROP_TYPES [CROP_TYPES ...]]
                        default_value: ['maize_01', 'maize_02']
  --litters LITTERS     default_value: 0
  --litter_types [LITTER_TYPES [LITTER_TYPES ...]]
                        default_value: ['ale', 'beer', 'coke_can',
                        'retro_pepsi_can']
  --weeds WEEDS         default_value: 0
  --weed_types [WEED_TYPES [WEED_TYPES ...]]
                        default_value: ['nettle', 'unknown_weed', 'dandelion']
  --ghost_objects GHOST_OBJECTS
                        default_value: False
  --location_markers LOCATION_MARKERS
                        default_value: False
  --load_from_file LOAD_FROM_FILE
                        default_value: None
  --seed SEED           default_value: -1
  --show_map            Show map after generation.
  ```
</details>

## Sample Worlds
In the [config folder](config/), config files to generate sample worlds are located. The parameters are chosen to match [the task description](https://www.fieldrobot.com/event/index.php/contest/)

Worlds for the Field Robot Event 2022:
| Name | Description |
|:---- |:----------- |
| *fre22_task_navigation* | Task navigation, curved rows that get more difficult (eg. have more and larger holes) to the left |
| *fre22_task_mapping* | Task mapping, field with random holes, bottles and weeds spread throughout the field. The cans, bottles and weeds have no collision box and are static. <br /><sub>This world needs dandelion models which were only distributed among competitors of the Field Robot Event 2022. They are not uploaded to Github because they cannot be open-sourced. If you don't have access to these models, check out the `fre21_task_3` worlds.</sub>|

Worlds for the Field Robot Event 2021:
| Name | Description |
|:---- |:----------- |
| *fre21_task_1* | Task 1, curved rows without holes |
| *fre21_task_2* | Task 2, straight rows with holes |
| *fre21_task_3* | Task 3, similar crop rows as in task_2 but with cans, bottles and weeds spread throughout the field. The cans, bottles and weeds have no collision box and are static. |
| *fre21_task_4* | Task 4, similar crop rows as in task_2 but with cans, bottles and weeds spread throughout the field. The cans, bottles and weeds have a collision box and can be picked up. |

You can use these config files when generating worlds, e.g.:
```commandline
ros2 run virtual_maize_field generate_world fre22_task_navigation_mini
```

## Launching and using generated worlds
The launch file to launch the worlds is called `simulation.launch`. You can launch the launch file by running `ros2 launch virtual_maize_field simulation.launch.py`. By default the launch file will launch `generated_world.world`. You can launch any world by using the `world_name` arg. e.g. `ros2 launch virtual_maize_field simulation.launch.py world_name:=simple_row_level_1.world`. The generated world will be saved in `$ROS_HOME/virtual_maize_field` (usually, this will be `~/.ros/virtual_maize_field`).

To add your own robot in the world, use the generated `robot_spawner.launch.py`. This launches your robot at the correct position in the generated world. Your launch file to launch your robot should look like (replace `<<robot_name>>` with your robot name):

```python
from __future__ import annotations

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from virtual_maize_field import get_spawner_launch_file


def generate_launch_description() -> LaunchDescription:
    robot_spawner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_spawner_launch_file()]),
        launch_arguments={"robot_name": "<<robot_name>>"}.items(),
    )

    return LaunchDescription([robot_spawner_launch])
```

Use the function `get_driving_pattern_file()` to get the path of the generated driving pattern and `get_markers_file` to get the path of the generated markers:

```python
from __future__ import annotations

from csv import reader
from pathlib import Path

from virtual_maize_field import get_driving_pattern_file, get_markers_file

def read_driving_pattern() -> None:
    driving_pattern = Path(get_driving_pattern_file()).read_text("utf-8")
    print(f"The driving pattern is {driving_pattern}")

def read_markers_file() -> None:
    with open(get_markers_file(), "r") as f:
        reader = csv.reader(f)

        # Skip header
        next(reader)

        for row in reader:
            print(f"Position {row[2]}: x={float(row[0]):.3f} y={float(row[1]):.3f}")
```

## License
Virtual Maize Field is copyright (C) 2021 *Farm Technology Group of Wageningen University & Research* and *Kamaro Engineering e.V.* and licensed under [![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0).

### Models
| Name | Path | Copyright | License |
|:---- |:---- |:--------- |:------- |
| [Maize 01](models/maize_01/model.config) | `models/maize_01/` | 2021 *Kamaro Engineering e.V.* | [![License: CC BY-SA 4.0](https://img.shields.io/badge/License-CC%20BY--SA%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-sa/4.0/) |
| [Maize 02](models/maize_02/model.config) | `models/maize_02/` | 2021 *Kamaro Engineering e.V.* | [![License: CC BY-SA 4.0](https://img.shields.io/badge/License-CC%20BY--SA%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-sa/4.0/) |
| [Stone 01](models/stone_01/model.config) | `models/stone_01/` | 2020 *Andrea Spognetta* | [![License: CC BY-NC 4.0](https://img.shields.io/badge/License-CC%20BY--NC%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-nc/4.0/) |
| [Stone 02](models/stone_02/model.config) | `models/stone_02/` | 2014 *Sascha Henrichs* | [![License: CC BY 4.0](https://img.shields.io/badge/License-CC%20BY%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by/4.0/) |
| [Ale](models/ale/model.config) | `models/ale/` | 2017 *elouisetrewartha* | [![License: CC BY-NC 4.0](https://img.shields.io/badge/License-CC%20BY--NC%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-nc/4.0/) |
| [Beer](models/beer/model.config) | `models/beer/` | ? *Maurice Fallon* | ? |
| [Coke Can](models/coke_can/model.config) | `models/coke_can/` | ? *John Hsu* | ? |
| [Nettle](models/nettle/model.config) | `models/nettle/` | 2019 *LadyIReyna* | [![License: CC BY-NC 4.0](https://img.shields.io/badge/License-CC%20BY--NC%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-nc/4.0/) |
| [Retro Pepsi Can](models/retro_pepsi_can/model.config) | `models/retro_pepsi_can/` | 2018 *FWTeastwood* | [![License: CC BY 4.0](https://img.shields.io/badge/License-CC%20BY%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by/4.0/) |
| [Unknown Weed](models/unknown_weed/model.config) | `models/unknown_weed/` | 2016 *aaron_nerlich* | [![License: CC BY 4.0](https://img.shields.io/badge/License-CC%20BY%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by/4.0/) |

### Textures
| Name | Path | Copyright | License |
|:---- |:---- |:--------- |:------- |
| [grass](https://cc0textures.com/view?id=Ground003) | [`Media/models/materials/textures/`](Media/models/materials/textures/) | 2018 *CC0Textures.com* | [![License: CC0-1.0](https://img.shields.io/badge/License-CC0%201.0-lightgrey.svg)](http://creativecommons.org/publicdomain/zero/1.0/) |
