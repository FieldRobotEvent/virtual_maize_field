#!/usr/bin/env python3

import argparse
import inspect
import json
from datetime import datetime

import numpy as np

from virtual_maize_field.world_generator import (
    AVAILABLE_CROP_TYPES,
    AVAILABLE_LITTER_TYPES,
    AVAILABLE_OBSTACLES,
    AVAILABLE_WEED_TYPES,
)

AVAILABLE_ILANDS = []
AVAILABLE_SEGMENTS = ["straight", "curved", "island"]


class WorldDescription:
    def __init__(
        self,
        row_length=12.0,
        rows_curve_budget=np.pi / 2,
        row_width=0.75,
        rows_count=6,
        row_segments=",".join(AVAILABLE_SEGMENTS[:2]),
        row_segment_straight_length_min=1,
        row_segment_straight_length_max=2.5,
        row_segment_curved_radius_min=3.0,
        row_segment_curved_radius_max=10.0,
        row_segment_curved_arc_measure_min=0.3,
        row_segment_curved_arc_measure_max=1.0,
        row_segment_island_radius_min=1.0,
        row_segment_island_radius_max=3.0,
        ground_resolution=0.02,
        ground_elevation_max=0.2,
        ground_headland=2.0,
        ground_ditch_depth=0.3,
        plant_spacing_min=0.13,
        plant_spacing_max=0.19,
        plant_height_min=0.3,
        plant_height_max=0.6,
        plant_radius=0.3,
        plant_radius_noise=0.05,
        plant_placement_error_max=0.02,
        plant_mass=0.3,
        hole_prob="0.06,0.06,0.04,0.04,0.0,0.0",
        hole_size_max="7,5,5,3,0,0",
        crop_types=",".join(list(AVAILABLE_CROP_TYPES.keys())),
        litters=0,
        litter_types=",".join(list(AVAILABLE_LITTER_TYPES.keys())),
        weeds=0,
        weed_types=",".join(list(AVAILABLE_WEED_TYPES.keys())),
        ghost_objects=False,
        location_markers=False,
        load_from_file=None,
        seed=-1,
    ):

        row_segments = row_segments.split(",")
        hole_prob = self.unpack_param(rows_count, hole_prob)
        hole_size_max = self.unpack_param(rows_count, hole_size_max)

        # accept array or single param

        if seed == -1:
            seed = int(datetime.now().timestamp() * 1000) % 8192

        for k, v in locals().items():
            self.__setattr__(k, v)

        np.random.seed(self.seed)

        if self.load_from_file is not None:
            self.load()
        else:
            self.random_description()

    def unpack_param(self, rows, value):
        if "," in str(value):
            array = value.split(",")
            if len(array) != rows:
                raise argparse.ArgumentError(
                    None,
                    "List argument must either be scalar or have one value for each row. See row_count.",
                )
        else:
            array = [value] * rows
        array = list(map(float, array))
        return array

    def random_description(self):
        self.structure = dict()
        self.structure["params"] = {
            "ground_resolution": self.ground_resolution,
            "ground_elevation_max": self.ground_elevation_max,
            "ground_headland": self.ground_headland,
            "ground_ditch_depth": self.ground_ditch_depth,
            "plant_spacing_min": self.plant_spacing_min,
            "plant_spacing_max": self.plant_spacing_max,
            "plant_height_min": self.plant_height_min,
            "plant_height_max": self.plant_height_max,
            "plant_radius": self.plant_radius,
            "plant_radius_noise": self.plant_radius_noise,
            "plant_placement_error_max": self.plant_placement_error_max,
            "plant_mass": self.plant_mass,
            "hole_prob": self.hole_prob,
            "hole_size_max": self.hole_size_max,
            "crop_types": self.crop_types,
            "litter_types": self.litter_types,
            "litters": self.litters,
            "weed_types": self.weed_types,
            "weeds": self.weeds,
            "ghost_objects": self.ghost_objects,
            "location_markers": self.location_markers,
            "seed": self.seed,
        }

        self.structure["segments"] = []
        current_row_length = 0
        current_curve = 0

        while current_row_length < self.row_length:
            # Choose rendom segment
            segment_name = np.random.choice(self.row_segments)

            if segment_name == "straight":
                length = (
                    np.random.rand()
                    * (
                        self.row_segment_straight_length_max
                        - self.row_segment_straight_length_min
                    )
                    + self.row_segment_straight_length_min
                )

                segment = {"type": "straight", "length": length}

                current_row_length += length

            elif segment_name == "curved":
                radius = (
                    np.random.rand()
                    * (
                        self.row_segment_curved_radius_max
                        - self.row_segment_curved_radius_min
                    )
                    + self.row_segment_curved_radius_min
                )
                arc_measure = (
                    np.random.rand()
                    * (
                        self.row_segment_curved_arc_measure_max
                        - self.row_segment_curved_arc_measure_min
                    )
                    + self.row_segment_curved_arc_measure_min
                )
                curve_dir = np.random.randint(2)
                if current_curve + arc_measure > self.rows_curve_budget:
                    curve_dir = 1
                elif current_curve - arc_measure < -self.rows_curve_budget:
                    curve_dir = 0

                segment = {
                    "type": "curved",
                    "radius": radius,
                    "curve_dir": curve_dir,
                    "arc_measure": arc_measure,
                }

                current_row_length += (
                    arc_measure * (self.rows_count * self.row_width + radius) / 2
                )
                current_curve = arc_measure if not curve_dir else -arc_measure

            elif segment_name == "island":
                radius = (
                    np.random.rand()
                    * (
                        self.row_segment_island_radius_max
                        - self.row_segment_island_radius_min
                    )
                    + self.row_segment_island_radius_min
                )
                island_row = np.random.randint(self.rows_count - 1) + 1

                segment = {
                    "type": "island",
                    "radius": radius,
                    "island_model": None,
                    "island_model_radius": radius,
                    "island_row": island_row,
                }

                # TODO
                current_row_length += 5

            else:
                raise ValueError("Unknown segment type. [" + segment_name + "]")

            self.structure["segments"].append(segment)

    def __str__(self):
        return json.dumps(self.structure, indent=2)

    def load(self):
        self.structure = json.load(open(self.load_from_file))

    def save(self, path):
        json.dump(self.structure, open(path, "w"), indent=2)


if __name__ == "__main__":
    # get the possible arguments of the generator and default values
    argspec = inspect.getfullargspec(WorldDescription.__init__)
    possible_kwargs = argspec.args[1:]
    defaults = argspec.defaults

    # construct an ArgumentParser that takes these arguments
    parser = argparse.ArgumentParser(
        description="Generate the json description for a virtual maize field."
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
    pk = WorldDescription(**args)

    print(pk)
