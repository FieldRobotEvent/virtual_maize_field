#!/usr/bin/env python3
from __future__ import annotations

from argparse import ArgumentError
from datetime import datetime
from json import dump, dumps, load

import numpy as np

from virtual_maize_field.world_generator.models import (
    AVAILABLE_MODELS,
    CROP_MODELS,
    LITTER_MODELS,
    OBSTACLE_MODELS,
    WEED_MODELS,
)

AVAILABLE_ILANDS = []
AVAILABLE_SEGMENTS = ["straight", "curved", "sincurved", "island"]


class WorldDescription:
    def __init__(
        self,
        row_length: float = 12.0,
        rows_curve_budget: float = np.pi / 2,
        row_width: float = 0.75,
        rows_count: int = 6,
        row_segments: list[str] = AVAILABLE_SEGMENTS[:2],
        row_segment_straight_length_min: int = 0.5,
        row_segment_straight_length_max: float = 1,
        row_segment_sincurved_offset_min: float = 0.5,
        row_segment_sincurved_offset_max: float = 1.5,
        row_segment_sincurved_length_min: float = 3,
        row_segment_sincurved_length_max: float = 5,
        row_segment_curved_radius_min: float = 3.0,
        row_segment_curved_radius_max: float = 10.0,
        row_segment_curved_arc_measure_min: float = 0.3,
        row_segment_curved_arc_measure_max: float = 1.0,
        row_segment_island_radius_min: float = 1.0,
        row_segment_island_radius_max: float = 3.0,
        ground_resolution: float = 0.02,
        ground_elevation_max: float = 0.2,
        ground_headland: float = 2.0,
        ground_ditch_depth: float = 0.3,
        plant_spacing_min: float = 0.13,
        plant_spacing_max: float = 0.19,
        plant_height_min: float = 0.3,
        plant_height_max: float = 0.6,
        plant_radius: float = 0.3,
        plant_radius_noise: float = 0.05,
        plant_placement_error_max: float = 0.02,
        plant_mass: float = 0.3,
        hole_prob: float | list[float] = [0.06, 0.06, 0.04, 0.04, 0.0, 0.0],
        hole_size_max: int | list[int] = [7, 5, 5, 3, 0, 0],
        crop_types: list[str] = list(CROP_MODELS.keys()),
        litters: int = 0,
        litter_types: list[str] = list(LITTER_MODELS.keys()),
        weeds: int = 0,
        weed_types: list[str] = list(WEED_MODELS.keys()),
        ghost_objects: bool = False,
        location_markers: bool = False,
        load_from_file: str | None = None,
        seed: int = -1,
        **kwargs,
    ) -> None:
        crop_types = self.unpack_model_types(crop_types)
        litter_types = self.unpack_model_types(litter_types)
        weed_types = self.unpack_model_types(weed_types)

        hole_prob = self.unpack_param(rows_count, hole_prob)
        hole_size_max = self.unpack_param(rows_count, hole_size_max)

        # accept array or single param

        if seed == -1:
            seed = int(datetime.now().timestamp() * 1000) % 8192

        for k, v in locals().items():
            self.__setattr__(k, v)

        self.rng = np.random.default_rng(seed)

        if self.load_from_file is not None:
            self.load()
        else:
            self.random_description()

    def unpack_model_types(self, model_types: list[str]) -> list[str]:
        for mt in model_types:
            if mt not in AVAILABLE_MODELS:
                raise ArgumentError(None, f"Error: Gazebo model {mt} is not valid!")
        return model_types

    def unpack_param(self, rows: int, value: list[float | int]) -> list[float | int]:
        if len(value) > 1:
            if len(value) != rows:
                raise ArgumentError(
                    None,
                    "List argument must either be scalar or have one value for each row. See row_count.",
                )
        else:
            value = [value] * rows
        return value

    def random_description(self) -> None:
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
            # Choose random segment
            segment_name = self.rng.choice(self.row_segments)

            if segment_name == "straight":
                length = (
                    self.rng.random()
                    * (
                        self.row_segment_straight_length_max
                        - self.row_segment_straight_length_min
                    )
                    + self.row_segment_straight_length_min
                )

                segment = {"type": "straight", "length": length}

                current_row_length += length

            elif segment_name == "sincurved":
                offset = (
                    self.rng.random()
                    * (
                        self.row_segment_sincurved_offset_max
                        - self.row_segment_sincurved_offset_min
                    )
                    + self.row_segment_sincurved_offset_min
                )
                length = (
                    self.rng.random()
                    * (
                        self.row_segment_sincurved_length_max
                        - self.row_segment_sincurved_length_min
                    )
                    + self.row_segment_sincurved_length_min
                )

                if current_row_length + length > self.row_length:
                    continue

                curve_dir = self.rng.integers(2)

                segment = {
                    "type": "sincurved",
                    "offset": offset,
                    "length": length,
                    "curve_dir": curve_dir,
                }

                current_row_length += length

            elif segment_name == "curved":
                radius = (
                    self.rng.random()
                    * (
                        self.row_segment_curved_radius_max
                        - self.row_segment_curved_radius_min
                    )
                    + self.row_segment_curved_radius_min
                )
                arc_measure = (
                    self.rng.random()
                    * (
                        self.row_segment_curved_arc_measure_max
                        - self.row_segment_curved_arc_measure_min
                    )
                    + self.row_segment_curved_arc_measure_min
                )
                curve_dir = self.rng.integers(2)
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
                    self.rng.random()
                    * (
                        self.row_segment_island_radius_max
                        - self.row_segment_island_radius_min
                    )
                    + self.row_segment_island_radius_min
                )
                island_row = self.rng.integers(self.rows_count - 1) + 1

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

    def __str__(self) -> str:
        return dumps(self.structure, indent=2)

    def load(self) -> None:
        self.structure = load(open(self.load_from_file))

    def save(self, path: str) -> None:
        dump(self.structure, open(path, "w"), indent=2)


if __name__ == "__main__":
    from world_generator.utils import parser_from_function

    parser = parser_from_function(WorldDescription.__init__)

    # get a dict representation of the arguments and call our constructor with them as kwargs
    args = vars(parser.parse_args())
    args = {k: v for k, v in args.items() if v is not None}
    pk = WorldDescription(**args)

    print(pk)
