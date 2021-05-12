#!/usr/bin/env python3

import rospkg
import argparse
import inspect
import os
import cv2
from matplotlib import pyplot as plt

from field_2d_generator import Field2DGenerator
from world_description import WorldDescription

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
    
    # save mini_map
    mini_map_path = os.path.join(pkg_path, "virtual_maize_field_mini_map.png")
    fgen.map.savefig(mini_map_path, dpi=1000)
