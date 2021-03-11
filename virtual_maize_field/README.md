# Virtual Maize Field
This is a package to procedurally generate randomized fields with rows of plants for Gazebo.

![Screenshot of a generated map with maize plants and pumpkins](./misc/screenshot.png)

## Installation
This package has been tested on `ROS melodic` and `ROS noetic`.

Additional you'll need the following packages:
```bash
# melodic
sudo apt install ros-melodic-gazebo-ros-pkgs python3-pip
sudo pip3 install -U jinja2 rospkg

# noetic
sudo apt install ros-noetic-gazebo-ros-pkgs \
                 python3-jinja2
```

## Generating field worlds
This package includes a script (`scripts/generate_world.py`) that can generate randomized agricultural worlds. All parameters are optional and have default values. You can call the script using
```bash
rosrun virtual_maize_field generate_world.py --num_rows_left=2 --num_rows_right=2 --num_plant_pairs=20 --max_angle_variation=0.15 --types=maize_01,maize_02 --row_width 1.0 --plant_offset 0.5
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
                         [--seed SEED]
                         [--types TYPES]

generate a world full of cylinders

optional arguments:
  -h, --help            show this help message and exit
  --plant_radius PLANT_RADIUS
                        default_value: 0.15
  --row_width ROW_WIDTH
                        default_value: 0.7
  --plant_offset PLANT_OFFSET
                        default_value: 0.6
  --max_angle_variation MAX_ANGLE_VARIATION
                        default_value: 0.08726646259971647
  --num_plant_pairs NUM_PLANT_PAIRS
                        default_value: 5
  --num_rows_left NUM_ROWS_LEFT
                        default_value: 2
  --num_rows_right NUM_ROWS_RIGHT
                        default_value: 3
  --plant_height PLANT_HEIGHT
                        default_value: 0.3
  --plant_mass PLANT_MASS
                        default_value: 5.0
  --radius_noise_range RADIUS_NOISE_RANGE
                        default_value: 0.05
  --seed SEED           default_value: None
  --types TYPES         default_value: pumpkin_01,pumpkin_02,pumpkin_03,cylinder,maize_01,maize_02,beans_01
```

## License info
Simple World is copyright (C) 2021 *Farm Technology Group of Wageningen University & Research* and *Kamaro Engineering e.V.* and licensed under GPLv3 (see [`LICENSE`](LICENSE)).
