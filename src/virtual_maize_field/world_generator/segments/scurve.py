from __future__ import annotations

from typing import Any

import numpy as np

from .base import BaseSegment


class SCurvedSegment(BaseSegment):
    def __init__(
        self,
        start_p: np.ndarray,
        start_dir: np.ndarray,
        plant_params: dict[str, Any],
        offset: float,
        length: float,
        curve_dir: int,
        step_size: float = 0.01,
        step_range: float = 50.0,
        steepness_step_size: float = 0.01,
        max_length_difference: float = 0.01,
        max_steps: int = 1e7,
        rng: np.random.Generator = np.random.default_rng(),
    ):
        super().__init__(start_p, start_dir, plant_params, rng)

        self.offset = offset
        self.length = length
        self.curve_dir = curve_dir

        # Calculate coordinates
        coordinates = []

        current_steepness = steepness_step_size
        current_length = 0
        steps = 0

        # TODO: make this more efficient
        while abs(current_length - (self.length / 2)) > max_length_difference:
            current_step = -step_range / 2
            coordinates = []

            while current_step < step_range / 2:
                current_step += step_size

                sigmoid_value = self.sigmoid(
                    current_step, self.offset, current_steepness
                )

                if sigmoid_value > 0.01 and sigmoid_value < self.offset - 0.01:
                    coordinates.append([sigmoid_value, current_step])

                steps += 1

                if steps > max_steps:
                    raise ValueError(
                        f"Cannot solve sigmoid function to have a line length of {self.length}!"
                    )

            coordinates = np.array(coordinates)
            current_length = self.coordinates_2_distance(coordinates)
            current_steepness += steepness_step_size

        if self.curve_dir == 0:
            coordinates[:, 0] *= -1.0

        # Flip coordinates for the back curve
        coordinates_co = coordinates.copy()
        coordinates_co -= coordinates_co[0]

        coordinates_ci = coordinates_co.copy()
        coordinates_ci[:, 0] = np.flip(coordinates_ci[:, 0], axis=0)
        coordinates_ci[:, 1] += coordinates_ci[:, 1].max()

        self.coordinates = np.concatenate((coordinates_co, coordinates_ci), axis=0)

    def end(self):
        end_p = np.empty_like(self.start_p)

        for i in range(len(self.start_p)):
            end_p[i] = self.start_p[i] + self.start_dir * self.coordinates[-1]

        return end_p, self.start_dir

    def get_plant_row(self, row_number, offset):
        start = self.start_p[row_number]
        c = self.bounded_gaussian.get(offset)

        cur_placement = start + self.start_dir * c
        placements = [cur_placement]

        row_coordinates = self.coordinates.copy() + cur_placement

        spacing = 0
        for i in range(1, row_coordinates.shape[0]):
            spacing += np.linalg.norm(row_coordinates[i - 1, :] - row_coordinates[i, :])

            if spacing >= self.plant_params["plant_spacing_min"]:
                placements.append(row_coordinates[i, :])
                spacing = 0

        return np.array(placements), spacing

    def racing_line(self, row_number, spacing):
        super().racing_line(row_number, spacing)

        start = self.start_p[row_number] + 0.5 * (
            self.start_p[row_number + 1] - self.start_p[row_number]
        )
        row_coordinates = self.coordinates.copy() + start

        line = [start]

        distance = 0
        for i in range(1, row_coordinates.shape[0]):
            distance += np.linalg.norm(
                row_coordinates[i - 1, :] - row_coordinates[i, :]
            )

            if distance >= spacing:
                line.append(row_coordinates[i, :])

        return np.array(line)

    def render(self):
        super().render()

    @staticmethod
    def sigmoid(
        x: float, max_value: float, steepness: float, midpoint: float = 0.0
    ) -> float:
        z = np.exp(-steepness * (x - midpoint))
        sig = max_value / (1 + z)
        return sig

    @staticmethod
    def coordinates_2_distance(coordinates: np.ndarray) -> float:
        distance = 0.0
        for i in range(1, coordinates.shape[0]):
            distance += np.linalg.norm(coordinates[i - 1, :] - coordinates[i, :])
        return distance
