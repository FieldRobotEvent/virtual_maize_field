#!/usr/bin/env python3

import rospkg
import argparse
import inspect

import os
import shutil

import cv2
from matplotlib import pyplot as plt
import yaml

from field_2d_generator import Field2DGenerator
from world_description import WorldDescription

if __name__ == "__main__":
    # get the possible arguments of the generator and default values
    argspec = inspect.getfullargspec(WorldDescription.__init__)
    possible_kwargs = argspec.args[1:]
    defaults = argspec.defaults

    # construct an ArgumentParser that takes these arguments
    parser = argparse.ArgumentParser(description="Generate a virtual maize field world for gazebo")
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
    heightmap_path = os.path.join(pkg_path, "Media/models/virtual_maize_field_heightmap.png")
    cv2.imwrite(heightmap_path, fgen.heightmap)

    # clear the gazbeo cache for old heightmap
    home_dir = os.path.expanduser("~")
    gazebo_cache_pkg = os.path.join(home_dir, ".gazebo/paging/virtual_maize_field_heightmap")
    if os.path.isdir(gazebo_cache_pkg):
        shutil.rmtree(gazebo_cache_pkg)

    # save mini_map
    minimap_path = os.path.join(pkg_path, "generated_minimap.png")
    fgen.minimap.savefig(minimap_path, dpi=100)
    
    # save the start location
    yaml_path = os.path.join(pkg_path, "config/spawn_location.yaml")
    
    with open(yaml_path, 'w') as outfile:
        # fgen
        data = {}
        data['spawn_position_x'] = float(fgen.start_loc[0][0])
        data['spawn_position_y'] = float(fgen.start_loc[0][1])
        data['spawn_position_z'] = 1
        data['spawn_position_yaw'] = 0
        yaml.dump(data, outfile, default_flow_style=False)    
