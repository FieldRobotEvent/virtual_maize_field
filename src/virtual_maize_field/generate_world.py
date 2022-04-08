#!/usr/bin/env python3
from __future__ import annotations

from csv import writer as csv_writer
from pathlib import Path
from shutil import rmtree

import cv2
import numpy as np
import rospkg
import yaml

from virtual_maize_field.world_generator.field_2d_generator import Field2DGenerator
from virtual_maize_field.world_generator.world_description import WorldDescription

LAUNCH_FILE_TEMPLATE = """<?xml version="1.0"?>
<launch>
    <!-- Spawn Robot -->
    <arg name="robot_name" default="robot_model" />
    <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model"
    args="-urdf -model $(arg robot_name) -param robot_description -x {x} -y {y} -z {x} -R 0 -P 0 -Y {yaw}" /> 
</launch>
"""


class WorldGenerator:
    def __init__(self, **kwargs) -> None:
        wd = WorldDescription(**kwargs)
        self.fgen = Field2DGenerator(wd)

        self.pkg_path = Path(rospkg.RosPack().get_path("virtual_maize_field"))

    def generate(self) -> None:
        """
        Generate the template and write it to a file.
        """
        generated_sdf, heightmap = self.fgen.generate()

        sdf_file = self.pkg_path / "worlds/generated.world"
        with sdf_file.open("w") as f:
            f.write(generated_sdf)

        # Save heightmap
        heightmap_file = (
            self.pkg_path / "Media/models/virtual_maize_field_heightmap.png"
        )
        cv2.imwrite(str(heightmap_file), self.fgen.heightmap)

    def clear_gazebo_cache(self) -> None:
        """
        Clear the Gazebo cache for old heightmap.
        """
        gazebo_cache_pkg = Path.home() / ".gazebo/paging/virtual_maize_field_heightmap"
        if gazebo_cache_pkg.is_dir():
            rmtree(gazebo_cache_pkg)

    def save_minimap(self) -> None:
        minimap_file = self.pkg_path / "map/map.png"
        self.fgen.minimap.savefig(str(minimap_file), dpi=100)

    def save_marker_file(self) -> None:
        marker_file = self.pkg_path / "map/markers.csv"
        with marker_file.open("w") as f:
            writer = csv_writer(f)
            header = ["X", "Y", "kind"]
            writer.writerow(header)
            if self.fgen.marker_a_loc.shape[0] != 0:
                writer.writerow(
                    [
                        self.fgen.marker_a_loc[0][0],
                        self.fgen.marker_a_loc[0][1],
                        "location_marker_a",
                    ]
                )
                writer.writerow(
                    [
                        self.fgen.marker_b_loc[0][0],
                        self.fgen.marker_b_loc[0][1],
                        "location_marker_b",
                    ]
                )

    def save_complete_map(self) -> None:
        complete_map_file = self.pkg_path / "map/map.csv"
        with complete_map_file.open("w") as f:
            writer = csv_writer(f)
            header = ["X", "Y", "kind"]
            writer.writerow(header)

            # marker
            if self.fgen.marker_a_loc.shape[0] != 0:
                writer.writerow(
                    [
                        self.fgen.marker_a_loc[0][0],
                        self.fgen.marker_a_loc[0][1],
                        "location_marker_a",
                    ]
                )
                writer.writerow(
                    [
                        self.fgen.marker_b_loc[0][0],
                        self.fgen.marker_b_loc[0][1],
                        "location_marker_b",
                    ]
                )

            for elm in self.fgen.weed_placements:
                writer.writerow([elm[0], elm[1], "weed"])

            for elm in self.fgen.litter_placements:
                writer.writerow([elm[0], elm[1], "litter"])

            for elm in self.fgen.crop_placements:
                writer.writerow([elm[0], elm[1], "crop"])

    def save_launch_file(self) -> None:
        launch_file = self.pkg_path / "launch/robot_spawner.launch"
        with launch_file.open("w") as f:
            content = LAUNCH_FILE_TEMPLATE.format(
                x=float(self.fgen.start_loc[0][0]) + np.random.rand() * 0.1 - 0.05,
                y=float(self.fgen.start_loc[0][1]) + np.random.rand() * 0.1 - 0.05,
                z=0.7,
                yaw=1.5707963267948966 + np.random.rand() * 0.1 - 0.05,
            )
            f.write(content)

    @classmethod
    def from_config_file(cls, config_file: Path) -> WorldGenerator:
        with config_file.open("r") as f:
            config = yaml.safe_load(f)
        return cls(**config)


def main() -> None:
    from argparse import ArgumentParser
    from inspect import getfullargspec

    pkg_path = Path(rospkg.RosPack().get_path("virtual_maize_field"))

    # Dynamically create argument parser with world description parameters
    argspec = getfullargspec(WorldDescription.__init__)
    possible_kwargs = argspec.args[1:]
    defaults = argspec.defaults

    parser = ArgumentParser(
        description="Generate a virtual maize field world for Gazebo."
    )
    parser.add_argument(
        "config_file",
        nargs="?",
        type=str,
        help="Config file name in the config folder",
        default=None,
        choices=[f.stem for f in (pkg_path / "config").glob("*.yaml")],
    )
    for argname, default in zip(possible_kwargs, defaults):
        # we analyze the default value's type to guess the type for that argument
        parser.add_argument(
            "--" + argname,
            type=type(default),
            help="default_value: {}".format(default),
            required=False,
        )

    args = parser.parse_args()

    if args.config_file:
        if ".yaml" in args.config_file:
            config_file_path = pkg_path / "config" / args.config_file
        else:
            config_file_path = pkg_path / "config" / (args.config_file + ".yaml")

        if not config_file_path.is_file():
            print(f"ERROR: cannot find config: '{config_file_path}'!")
            exit(1)

        generator = WorldGenerator.from_config_file(config_file_path)
    else:
        # Get a dict representation of the arguments and call our constructor with them as kwargs
        args = vars(parser.parse_args())
        args = {k: v for k, v in args.items() if v is not None}
        generator = WorldGenerator(**args)

    generator.generate()
    generator.clear_gazebo_cache()
    generator.save_minimap()
    generator.save_marker_file()
    generator.save_complete_map()
    generator.save_launch_file()


if __name__ == "__main__":
    main()
