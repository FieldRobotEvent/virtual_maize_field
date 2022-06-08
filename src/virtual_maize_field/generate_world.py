#!/usr/bin/env python3
from __future__ import annotations

import importlib.resources
from csv import writer as csv_writer
from pathlib import Path
from shutil import rmtree

import cv2
import numpy as np
import rospkg
import yaml
from jinja2 import Template

from virtual_maize_field import world_generator
from virtual_maize_field.world_generator.field_2d_generator import Field2DGenerator
from virtual_maize_field.world_generator.world_description import WorldDescription


class WorldGenerator:
    def __init__(self, **kwargs) -> None:
        self.wd = WorldDescription(**kwargs)

        self.fgen = Field2DGenerator(self.wd)
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

    def save_gt_minimap(self) -> None:
        minimap_file = self.pkg_path / "gt/map.png"
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

    def save_gt_map(self) -> None:
        complete_map_file = self.pkg_path / "gt/map.csv"
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
        launch_file_template = Template(
            importlib.resources.read_text(
                world_generator, "robot_spawner.launch.template"
            )
        )
        launch_file = self.pkg_path / "launch/robot_spawner.launch"

        with launch_file.open("w") as f:
            content = launch_file_template.render(
                x=float(self.fgen.start_loc[0][0]) + self.wd.rng.random() * 0.1 - 0.05,
                y=float(self.fgen.start_loc[0][1]) + self.wd.rng.random() * 0.1 - 0.05,
                z=0.35,
                roll=0,
                pitch=0,
                yaw=1.5707963267948966 + self.wd.rng.random() * 0.1 - 0.05,
            )
            f.write(content)

    @classmethod
    def from_config_file(cls, config_file: Path) -> WorldGenerator:
        with config_file.open("r") as f:
            config = yaml.safe_load(f)
        return cls(**config)


def main() -> None:
    from world_generator.utils import parser_from_function

    pkg_path = Path(rospkg.RosPack().get_path("virtual_maize_field"))

    parser = parser_from_function(
        WorldDescription.__init__,
        description="Generate a virtual maize field world for Gazebo.",
    )
    parser.add_argument(
        "config_file",
        nargs="?",
        type=str,
        help="Config file name in the config folder",
        default=None,
        choices=[f.stem for f in (pkg_path / "config").glob("*.yaml")],
    )
    parser.add_argument(
        "--show_map", action="store_true", help="Show map after generation."
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
        args_dict = {k: v for k, v in vars(args).items() if v is not None}
        generator = WorldGenerator(**args_dict)

    generator.generate()
    generator.clear_gazebo_cache()
    generator.save_gt_minimap()
    generator.save_marker_file()
    generator.save_gt_map()
    generator.save_launch_file()

    # Show minimap after generation
    if args.show_map:
        generator.fgen.minimap.show()


if __name__ == "__main__":
    main()
