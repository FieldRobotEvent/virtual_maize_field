# Virtual Maize Field

<p float="left" align="middle">
  <img src="https://www.fieldrobot.com/event/wp-content/uploads/2021/01/FRE-logo-v02.png" width="250">
</p>
<p float="left" align="middle"> 
  <img src="https://www.wur.nl/upload/58340fb4-e33a-4d0b-af17-8d596fa93663_WUR_RGB_standard.png" width="250" style="margin: 10px;"> 
  <img src="https://www.uni-hohenheim.de/typo3conf/ext/uni_layout/Resources/Public/Images/uni-logo-en.svg" width="250" style="margin: 10px;">
  <img src="https://kamaro-engineering.de/wp-content/uploads/2015/03/Kamaro_Logo-1.png" width="250" style="margin: 10px;">
</p>
<p align="middle">
  <a href="https://github.com/psf/black"><img src="https://img.shields.io/badge/code%20style-black-000000.svg" alt="Code style: black"/></a>
  <a href="https://github.com/FieldRobotEvent/Virtual_Field_Robot_Event/discussions"><img src="https://img.shields.io/badge/contributions-welcome-brightgreen.svg?style=flat" alt="contributions welcome"/></a>
  <a href="https://www.gnu.org/licenses/gpl-3.0"><img src="https://img.shields.io/badge/License-GPLv3-blue.svg" alt="License: GPL v3"/></a>
</p>
<p align="middle">
  This is a package to procedurally generate randomized fields with rows of plants for Gazebo.
</p>

![Screenshot of a generated map with maize plants and pumpkins](./misc/screenshot_v3.0.png)

## Installation
This package has been tested on `ROS melodic` and `ROS noetic`.

Additional you'll need the following packages:
```bash
# melodic
rosdep install virtual_maize_field
sudo apt install python3-pip
sudo pip3 install -U jinja2 rospkg opencv-python matplotlib shapely

# noetic
rosdep install virtual_maize_field
```

## Generating new maize field worlds
This package includes a script (`scripts/generate_world.py`) that can generate randomized agricultural worlds. All parameters are optional and have default values. You can call the script using
```bash
rosrun virtual_maize_field generate_world.py
```
The resulting file will be placed in `worlds/generated.world`.
```
usage: generate_world.py [-h] [--row_length ROW_LENGTH] [--rows_curve_budget ROWS_CURVE_BUDGET] [--row_width ROW_WIDTH] [--rows_left ROWS_LEFT] [--rows_right ROWS_RIGHT] [--row_segments ROW_SEGMENTS]
                         [--row_segment_straight_length_min ROW_SEGMENT_STRAIGHT_LENGTH_MIN] [--row_segment_straight_length_max ROW_SEGMENT_STRAIGHT_LENGTH_MAX]
                         [--row_segment_curved_radius_min ROW_SEGMENT_CURVED_RADIUS_MIN] [--row_segment_curved_radius_max ROW_SEGMENT_CURVED_RADIUS_MAX]
                         [--row_segment_curved_arc_measure_min ROW_SEGMENT_CURVED_ARC_MEASURE_MIN] [--row_segment_curved_arc_measure_max ROW_SEGMENT_CURVED_ARC_MEASURE_MAX]
                         [--row_segment_island_radius_min ROW_SEGMENT_ISLAND_RADIUS_MIN] [--row_segment_island_radius_max ROW_SEGMENT_ISLAND_RADIUS_MAX] [--ground_resolution GROUND_RESOLUTION]
                         [--ground_elevation_max GROUND_ELEVATION_MAX] [--ground_headland GROUND_HEADLAND] [--ground_ditch_depth GROUND_DITCH_DEPTH] [--plant_spacing_min PLANT_SPACING_MIN]
                         [--plant_spacing_max PLANT_SPACING_MAX] [--plant_height_min PLANT_HEIGHT_MIN] [--plant_height_max PLANT_HEIGHT_MAX] [--plant_radius PLANT_RADIUS]
                         [--plant_radius_noise PLANT_RADIUS_NOISE] [--plant_placement_error_max PLANT_PLACEMENT_ERROR_MAX] [--plant_mass PLANT_MASS] [--hole_prob HOLE_PROB] [--hole_size_max HOLE_SIZE_MAX]
                         [--crop_types CROP_TYPES] [--litters LITTERS] [--litter_types LITTER_TYPES] [--weeds WEEDS] [--weed_types WEED_TYPES] [--ghost_objects GHOST_OBJECTS] [--location_markers LOCATION_MARKERS] [--load_from_file LOAD_FROM_FILE]
                         [--seed SEED]

Generate a virtual maize field world for gazebo

optional arguments:
  -h, --help            show this help message and exit
  --row_length ROW_LENGTH
                        default_value: 12.0
  --rows_curve_budget ROWS_CURVE_BUDGET
                        default_value: 1.5707963267948966
  --row_width ROW_WIDTH
                        default_value: 0.75
  --rows_left ROWS_LEFT
                        default_value: 2
  --rows_right ROWS_RIGHT
                        default_value: 2
  --row_segments ROW_SEGMENTS
                        default_value: straight,curved
  --row_segment_straight_length_min ROW_SEGMENT_STRAIGHT_LENGTH_MIN
                        default_value: 1
  --row_segment_straight_length_max ROW_SEGMENT_STRAIGHT_LENGTH_MAX
                        default_value: 2.5
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
  --hole_prob HOLE_PROB
                        default_value: 0.0
  --hole_size_max HOLE_SIZE_MAX
                        default_value: 7
  --crop_types CROP_TYPES
                        default_value: maize_01,maize_02
  --litters LITTERS     default_value: 0
  --litter_types LITTER_TYPES
                        default_value: ale,beer,coke_can,retro_pepsi_can
  --weeds WEEDS         default_value: 0
  --weed_types WEED_TYPES
                        default_value: nettle,unknown_weed
  --ghost_objects GHOST_OBJECTS
                        default_value: False
  --location_markers LOCATION_MARKERS
			default_value: False
  --load_from_file LOAD_FROM_FILE
                        default_value: None
  --seed SEED           default_value: -1
```

## Sample Worlds
In the script folder, bash files to generate sample worlds are located. The parameters are chosen to match the task description from https://www.fieldrobot.com/event/index.php/contest/
| Name | Parameters | Description |
|:---- |:--------- |:----------- |
|Task 1. The field contains curved rows. The robot has to drive through the row, into the next row and repeat this process till a 3-minute timer runs out, or till the robot has reached the end of the field.|
| *create_task_1.sh* | `--row_length 10 --rows_left 0 --rows_right 11 --rows_curve_budget 0.78539816339 --row_segments straight,curved --row_segment_curved_radius_min 4.0 --row_segment_curved_radius_max 5.0` | Task 1, curved rows without holes |
| *create_task_1_mini.sh* | `--row_length 5 --rows_left 0 --rows_right 5 --rows_curve_budget 0.78539816339 --row_segments straight,curved --row_segment_curved_radius_min 4.0 --row_segment_curved_radius_max 5.0` | A smaller version of task 1, requiring less computer power |
|Task 2. The field contains straight rows with missing plants. The Robot has to drive through the rows according to a given pattern. E.g. 3L – 2L – 2R – 1R – 5L – F.|
| *create_task_2.sh* | `--row_length 7 --rows_left 0 --rows_right 11 --row_segments straight --hole_prob 0.04 --hole_size_max 7` | Task 2, straight rows with holes |
| *create_task_2_mini.sh* | `--row_length 3.5 --rows_left 0 --rows_right 7 --row_segments straight --hole_prob 0.04 --hole_size_max 7` | A smaller version of task 2, requiring less computer power |
|Task 3. The field contains straight rows with missing plants. On the headland, two pillars, with a QR code are located. The QR code on one pillar says `Location¬_marker_A`, and the QR code on the other pillar says `Location_marker_B`. The location of these pillars is given by the map generator in the file `virtual_maize_field/map/markers.csv`. Throughout the field, there are 5 weeds and 5 cans/bottles. These objects are “ghost” objects, meaning that they are visible, but they don’t have a collision box and cannot be moved. These objects need to be mapped in the coordinate system of the two location markers. Afterwards, your robot needs to make a *.csv file with the locations of the weeds and the bottles/cans. This .csv file must be in the same format as the file located in `virtual_maize_field/map/example_pred_map.csv`. Using the script `virtual_maize_field/scripts/compare_maps.py`, the locations of your detections can be compared to the ground truth locations of the objects, and you will get a score for your map. In the location `virtual_maize_field/map/evaluation_map.png` you will find a map containing your detections, the ground truth detections, and your map score.|
| *create_task_3.sh* | `--row_length 7 --rows_left 0 --rows_right 11 --row_segments straight --hole_prob 0.04 --hole_size_max 7 --litters 5 --weeds 5 --ghost_objects true --location_markers true` | Task 3, similar crop rows as in task_2 but with cans, bottles and weeds spread throughout the field. The cans, bottles and weeds have no collision box and are static. |
| *create_task_3_mini.sh* | `--row_length 3.5 --rows_left 0 --rows_right 7 --row_segments straight --hole_prob 0.04 --hole_size_max 7 --litters 5 --weeds 5 --ghost_objects true --location_markers true` | A smaller version of task 3, requiring less computer power |
|Task 4. The field looks the same as in task 3, however, the weeds, cans, and bottles have a collision box and can be moved by your robot. At the beginning of the task, you will get a map of the field. The map is located in `virtual_maize_field/map/map.csv`, and `virtual_maize_field/map/map.png`. The map contains the ground truth location of all the plants, location markers, weeds, cans, and bottles. You can use this map for optimal path planning. During this task, the robot has to pick up the weeds and drop them on the headland of `Location_marker_A`. The bottles and cans also need to be picked up and dropped on the headland of `Location_maker_B`.|
| *create_task_4.sh* | `--row_length 7 --rows_left 0 --rows_right 11 --row_segments straight --hole_prob 0.04 --hole_size_max 7 --litters 5 --weeds 5 --location_markers true` | Task 4, similar crop rows as in task_2 but with cans, bottles and weeds spread throughout the field. The cans, bottles and weeds have a collision box and can be picked up. |
| *create_task_4_mini.sh* | `--row_length 3.5 --rows_left 0 --rows_right 7 --row_segments straight --hole_prob 0.04 --hole_size_max 7 --litters 5 --weeds 5 --location_markers true` | A smaller version of task 4, requiring less computer power |

## Launching worlds
The launch file to launch the worlds is called `simulation.launch`. You can launch the launch file by running `roslaunch virtual_maize_field jackal_simulation.launch`. By default the launch file will launch `generated_world.world`. You can launch any world by using the `world_name` arg. e.g. `roslaunch virtual_maize_field jackal_simulation.launch world_name:=simple_row_level_1.world`.

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
