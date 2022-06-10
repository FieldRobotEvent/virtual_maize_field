from __future__ import annotations

from typing import Any

import numpy as np

from .base import BaseSegment


class SinCurvedSegment(BaseSegment):
    def __init__(
        self,
        start_p: np.ndarray,
        start_dir: np.ndarray,
        plant_params: dict[str, Any],
        offset: float,
        length: float,
        curve_dir: int,
        step_size: float = 0.01,
        rng: np.random.Generator = np.random.default_rng(),
    ):
        super().__init__(start_p, start_dir, plant_params, rng)

        y_values = np.arange(0, length, step_size)
        x_values = offset / 2 + (offset / 2) * np.sin(
            -0.5 * np.pi + (2 * np.pi / length) * y_values
        )
        self.coordinates = np.stack((x_values, y_values), axis=1)

        if curve_dir == 0:
            self.coordinates[:, 0] *= -1.0

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
