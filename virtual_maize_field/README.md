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
  <a href="https://www.gnu.org/licenses/gpl-3.0"><img src="https://img.shields.io/badge/License-GPLv3-blue.svg" alt="License: GPL v3"/></a>
</p>
<p align="middle">
  This is a package to procedurally generate randomized fields with rows of plants for Gazebo.
</p>

![Screenshot of a generated map with maize plants and pumpkins](./misc/screenshot.png)

## Installation
This package has been tested on `ROS melodic` and `ROS noetic`.

Additional you'll need the following packages:
```bash
# melodic
rosdep install virtual_maize_field
sudo apt install python3-pip
sudo pip3 install -U jinja2 rospkg

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
usage: generate_world.py [-h] [--plant_radius PLANT_RADIUS]
                         [--row_width ROW_WIDTH]
                         [--plant_offset PLANT_OFFSET]
                         [--max_angle_variation MAX_ANGLE_VARIATION]
                         [--num_plant_pairs NUM_PLANT_PAIRS]
                         [--num_rows_left NUM_ROWS_LEFT]
                         [--num_rows_right NUM_ROWS_RIGHT]
                         [--plant_height PLANT_HEIGHT]
                         [--plant_mass PLANT_MASS]
                         [--radius_noise_range RADIUS_NOISE_RANGE]
                         [--position_div POSITION_DIV]
                         [--dropout DROPOUT]
                         [--seed SEED]
                         [--types TYPES]

Generate a virtual maize field world for gazebo

optional arguments:
  -h, --help            show this help message and exit
  --plant_radius PLANT_RADIUS
                        default_value: 0.15
  --row_width ROW_WIDTH
                        default_value: 0.75
  --plant_offset PLANT_OFFSET
                        default_value: 0.5
  --max_angle_variation MAX_ANGLE_VARIATION
                        default_value: 0.15
  --num_plant_pairs NUM_PLANT_PAIRS
                        default_value: 20
  --num_rows_left NUM_ROWS_LEFT
                        default_value: 2
  --num_rows_right NUM_ROWS_RIGHT
                        default_value: 2
  --plant_height PLANT_HEIGHT
                        default_value: 0.3
  --plant_mass PLANT_MASS
                        default_value: 5.0
  --radius_noise_range RADIUS_NOISE_RANGE
                        default_value: 0.05
  --position_div POSITION_DIV
                        default_value: 0.03
  --dropout DROPOUT     default_value: 0.0
  --seed SEED           default_value: None
  --types TYPES         default_value: cylinder,maize_01,maize_02
```

## Sample Worlds
| Name | Parameters | Description |
|:---- |:--------- |:----------- |
| *simple_row_level_0.world* | `--plant_radius=0.03 --max_angle_variation=0 --plant_height=0.75 --radius_noise_range=0 --position_div=0 --types=cylinder` | One row with grid based cylinders. |
| *simple_row_level_1.world* | `--max_angle_variation=0 --position_div=0` | One row with grid based plants. |
| *simple_row_level_2.world* | `--max_angle_variation=0` | One row with more natural plant placement. |
| *simple_row_level_3.world* | default | One row with little curvature. |
| *simple_row_level_4.world* | `--max_angle_variation=0.3` | One row with curvature. |
| *simple_row_level_5.world* | `--dropout=0.1` | Level 3 with gaps |
| *simple_row_level_6.world* | `--max_angle_variation=0.3 --dropout=0.1` | Level 4 with gaps |

## Launching worlds
The launch file to launch the worlds is called `simulation.launch`. You can launch the launch file by running `roslaunch virtual_maize_field simulation.launch`. By default the launch file will launch `generated_world.world`. You can launch any world by using the `world_name` arg. e.g. `roslaunch virtual_maize_field simulation.launch world_name:=simple_row_level_1.world`.

## License
Virtual Maize Field is copyright (C) 2021 *Farm Technology Group of Wageningen University & Research* and *Kamaro Engineering e.V.* and licensed under [![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0).

### Models
| Name | Path | Copyright | License |
|:---- |:---- |:--------- |:------- |
| [Maize 01](models/maize_01/model.config) | `models/maize_01/` | 2021 *Kamaro Engineering e.V.* | [![License: CC BY-SA 4.0](https://img.shields.io/badge/License-CC%20BY--SA%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-sa/4.0/) |
| [Maize 02](models/maize_02/model.config) | `models/maize_02/` | 2021 *Kamaro Engineering e.V.* | [![License: CC BY-SA 4.0](https://img.shields.io/badge/License-CC%20BY--SA%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-sa/4.0/) |
| [Stone 01](models/stone_01/model.config) | `models/stone_01/` | 2020 *Andrea Spognetta* | [![License: CC BY-NC 4.0](https://img.shields.io/badge/License-CC%20BY--NC%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-nc/4.0/) |
| [Stone 02](models/stone_02/model.config) | `models/stone_02/` | 2014 *Sascha Henrichs* | [![License: CC BY 4.0](https://img.shields.io/badge/License-CC%20BY%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by/4.0/) |
