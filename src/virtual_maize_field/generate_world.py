#!/usr/bin/env python3

import argparse
import csv
import inspect
import os
import shutil

import cv2
import numpy as np
import rospkg

from virtual_maize_field.world_generator.field_2d_generator import Field2DGenerator
from virtual_maize_field.world_generator.world_description import WorldDescription

SPAWNER_LAUNCH_FILE = """<?xml version="1.0"?>
<launch>
    <!-- Spawn Robot -->
    <arg name="robot_name" default="robot_model" />
    <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model"
    args="-urdf -model $(arg robot_name) -param robot_description -x {x} -y {y} -z {z} -R 0 -P 0 -Y {yaw}" /> 
</launch>"""

if __name__ == "__main__":
    # get the possible arguments of the generator and default values
    argspec = inspect.getfullargspec(WorldDescription.__init__)
    possible_kwargs = argspec.args[1:]
    defaults = argspec.defaults

    # construct an ArgumentParser that takes these arguments
    parser = argparse.ArgumentParser(
        description="Generate a virtual maize field world for gazebo"
    )
    for argname, default in zip(possible_kwargs, defaults):
        # we analyze the default value's type to guess the type for that argument
        parser.add_argument(
            "--" + argname,
            type=type(default),
            help="default_value: {}".format(default),
            required=False,
        )

    # get a dict representation of the arguments and call our constructor with them as kwargs
    args = vars(parser.parse_args())
    args = {k: v for k, v in args.items() if v is not None}
    wd = WorldDescription(**args)
    fgen = Field2DGenerator(wd)

    # generate the template and write it to a file
    pkg_path = rospkg.RosPack().get_path("virtual_maize_field")
    generated_sdf, heightmap = fgen.generate()
    sdf_path = os.path.join(pkg_path, "worlds/generated.world")
    with open(sdf_path, "w") as f:
        f.write(generated_sdf)
    # save heightmap
    heightmap_path = os.path.join(
        pkg_path, "Media/models/virtual_maize_field_heightmap.png"
    )
    cv2.imwrite(heightmap_path, fgen.heightmap)

    # clear the Gazebo cache for old heightmap
    home_dir = os.path.expanduser("~")
    gazebo_cache_pkg = os.path.join(
        home_dir, ".gazebo/paging/virtual_maize_field_heightmap"
    )
    if os.path.isdir(gazebo_cache_pkg):
        shutil.rmtree(gazebo_cache_pkg)

    # save mini_map
    minimap_path = os.path.join(pkg_path, "map/map.png")
    fgen.minimap.savefig(minimap_path, dpi=100)

    # marker file
    f_path = os.path.join(pkg_path, "map/markers.csv")
    with open(f_path, "w") as f:
        writer = csv.writer(f)
        header = ["X", "Y", "kind"]
        writer.writerow(header)
        if fgen.marker_a_loc.shape[0] != 0:
            writer.writerow(
                [fgen.marker_a_loc[0][0], fgen.marker_a_loc[0][1], "location_marker_a"]
            )

        if fgen.marker_b_loc.shape[0] != 0:
            writer.writerow(
                [fgen.marker_b_loc[0][0], fgen.marker_b_loc[0][1], "location_marker_b"]
            )

    # complete map
    f_path = os.path.join(pkg_path, "map/map.csv")
    with open(f_path, "w") as f:
        writer = csv.writer(f)
        header = ["X", "Y", "kind"]
        writer.writerow(header)

        # marker
        if fgen.marker_a_loc.shape[0] != 0:
            writer.writerow(
                [fgen.marker_a_loc[0][0], fgen.marker_a_loc[0][1], "location_marker_a"]
            )

        if fgen.marker_b_loc.shape[0] != 0:
            writer.writerow(
                [fgen.marker_b_loc[0][0], fgen.marker_b_loc[0][1], "location_marker_b"]
            )

        for elm in fgen.weed_placements:
            writer.writerow([elm[0], elm[1], "weed"])

        for elm in fgen.litter_placements:
            writer.writerow([elm[0], elm[1], "litter"])

        for elm in fgen.crop_placements:
            writer.writerow([elm[0], elm[1], "crop"])

    # save the start location in a launch file
    launch_path = os.path.join(pkg_path, "launch/robot_spawner.launch")
    with open(launch_path, "w") as launch_file:
        launch_file.write(
            SPAWNER_LAUNCH_FILE.format(
                x=float(fgen.start_loc[0][0]) + fgen.rng.random() * 0.1 - 0.05,
                y=float(fgen.start_loc[0][1]) + fgen.rng.random() * 0.1 - 0.05,
                z=0.7,
                yaw=1.5707963267948966 + fgen.rng.random() * 0.1 - 0.05,
            )
        )
