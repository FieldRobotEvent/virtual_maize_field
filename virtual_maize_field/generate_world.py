#!/usr/bin/env python3
from __future__ import annotations

import importlib.resources
from csv import writer as csv_writer
from os import environ
from pathlib import Path
from shutil import rmtree

import cv2
import yaml
from ament_index_python.packages import get_package_share_directory
from jinja2 import Template

from virtual_maize_field import world_generator
from virtual_maize_field.world_generator.field_2d_generator import Field2DGenerator
from virtual_maize_field.world_generator.world_description import WorldDescription


class WorldGenerator:
    def __init__(self, **kwargs) -> None:
        self.wd = WorldDescription(**kwargs)

        self.fgen = Field2DGenerator(self.wd)
        self.pkg_path = Path(get_package_share_directory("virtual_maize_field"))
        _ros_home_path = environ.get("ROS_HOME", str(Path.home() / ".ros"))
        self.cache_folder = Path(_ros_home_path) / "virtual_maize_field"
        self.cache_folder.mkdir(parents=True, exist_ok=True)

    def generate(self) -> None:
        """
        Generate the template and write it to a file.
        """
        generated_sdf, _ = self.fgen.generate(self.cache_folder)

        sdf_file = self.cache_folder / "generated.world"
        with sdf_file.open("w") as f:
            f.write(generated_sdf)

        # Save heightmap
        heightmap_file = self.cache_folder / "virtual_maize_field_heightmap.png"
        cv2.imwrite(str(heightmap_file), self.fgen.heightmap)

        print(f"Saved world (sdf) to {sdf_file}")

    def clear_gazebo_cache(self) -> None:
        """
        Clear the Gazebo cache for old heightmap.
        """
        gazebo_cache_pkg = Path.home() / ".gazebo/paging/virtual_maize_field_heightmap"
        if gazebo_cache_pkg.is_dir():
            rmtree(gazebo_cache_pkg)

    def generate_driving_pattern(self) -> None:
        # TODO: generate realistic pattern
        pattern = "S – 1L – 1R – 1L – 1R – 1L – 1R – 1L – 1R – 1L – 1R – F"
        pattern_file = self.cache_folder / "driving_pattern.txt"
        pattern_file.write_text(pattern)

    def save_gt_minimap(self) -> None:
        minimap_file = self.cache_folder / "gt_map.png"
        self.fgen.minimap.savefig(str(minimap_file), dpi=100)

        print(f"Saved ground truth minimap to {minimap_file}")

    def save_marker_file(self) -> None:
        marker_file = self.cache_folder / "markers.csv"
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

        print(f"Saved marker locations to {marker_file}")

    def save_gt_map(self) -> None:
        complete_map_file = self.cache_folder / "gt_map.csv"
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

        print(f"Saved ground truth locations to {complete_map_file}")

    def save_launch_file(self) -> None:
        launch_file_template = Template(
            importlib.resources.read_text(
                world_generator, "robot_spawner.launch.py.template"
            )
        )
        launch_file = self.cache_folder / "robot_spawner.launch.py"

        with launch_file.open("w") as f:
            content = launch_file_template.render(
                x=float(self.fgen.start_loc[0][0]) + self.wd.rng.random() * 0.1 - 0.05,
                y=float(self.fgen.start_loc[0][1]) + self.wd.rng.random() * 0.1 - 0.05,
                z=0.35 + self.wd.ground_ditch_depth,
                roll=0,
                pitch=0,
                yaw=1.5707963267948966 + self.wd.rng.random() * 0.1 - 0.05,
            )
            f.write(content)

        print(f"Saved robot spawner launch file to {launch_file}")

    @classmethod
    def from_config_file(cls, config_file: Path) -> WorldGenerator:
        with config_file.open("r") as f:
            config = yaml.safe_load(f)
        return cls(**config)


def main() -> None:
    from virtual_maize_field.world_generator.utils import parser_from_function

    pkg_path = Path(get_package_share_directory("virtual_maize_field"))

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
    generator.generate_driving_pattern()

    # Show minimap after generation
    if args.show_map:
        generator.fgen.minimap.show()


if __name__ == "__main__":
    main()
