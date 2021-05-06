#!/usr/bin/env python3

import numpy as np
import json
import argparse
import inspect
from datetime import datetime

AVAILABLE_TYPES = ["cylinder", "maize_01", "maize_02"]
AVAILABLE_OBSTACLES = ["box", "stone_01", "stone_02"]
AVAILABLE_ILANDS = []
AVAILABLE_SEGMENTS = ["straight", "curved", "iland"]


class WorldDescription:
    def __init__(
        self,
        row_length=12.0,
        row_width=0.75,
        rows_left=2,
        rows_right=2,
        rows_curve_budget=np.pi / 2,
        row_segments=",".join(AVAILABLE_SEGMENTS[:2]),
        row_segment_straight_length_min=1,
        row_segment_straight_length_max=2.5,
        row_segment_curved_radius_min=3.0,
        row_segment_curved_radius_max=10.0,
        row_segment_curved_arc_measure_min=0.3,
        row_segment_curved_arc_measure_max=1.0,
        row_segment_island_radius_min=1.0,
        row_segment_island_radius_max=3.0,
        ground_max_elevation=0.2,
        plant_spacing_min=0.13,
        plant_spacing_max=0.19,
        plant_height_min=0.3,
        plant_height_max=0.6,
        plant_radius=0.3,
        plant_radius_noise=0.05,
        plant_placement_error_max=0.02,
        plant_mass=0.3,
        plant_dropout=0.0,
        plant_types=",".join(AVAILABLE_TYPES[1:]),
        load_from_file=None,
        seed=-1,
    ):

        row_segments = row_segments.split(",")

        if seed == -1:
            seed = int(datetime.now().timestamp() * 1000) % 8192

        for k, v in locals().items():
            self.__setattr__(k, v)

        np.random.seed(self.seed)

        if self.load_from_file is not None:
            self.load()
        else:
            self.random_description()

    def random_description(self):
        self.structure = dict()
        self.structure["params"] = {
            "ground_max_elevation": self.ground_max_elevation,
            "plant_spacing_min": self.plant_spacing_min,
            "plant_spacing_max": self.plant_spacing_max,
            "plant_height_min": self.plant_height_min,
            "plant_height_max": self.plant_height_max,
            "plant_radius": self.plant_radius,
            "plant_radius_noise": self.plant_radius_noise,
            "plant_placement_error_max": self.plant_placement_error_max,
            "plant_mass": self.plant_mass,
            "plant_dropout": self.plant_dropout,
            "plant_types": self.plant_types,
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
                    arc_measure
                    * ((self.rows_left + self.rows_right) * self.row_width + radius)
                    / 2
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
                island_row = np.random.randint(self.rows_left + self.rows_right - 1) + 1

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