from __future__ import annotations

from typing import Any

import numpy as np
from matplotlib import pyplot as plt

from .base import BaseSegment


class StraightSegment(BaseSegment):
    def __init__(
        self,
        start_p: np.ndarray,
        start_dir: np.ndarray,
        plant_params: dict[str, Any],
        length: float,
        rng: np.random.RandomState = np.random.default_rng(),
    ):
        super().__init__(start_p, start_dir, plant_params, rng=rng)
        self.length = length

    def end(self):
        end_p = np.empty_like(self.start_p)

        for i in range(len(self.start_p)):
            end_p[i] = self.start_p[i] + self.start_dir * self.length

        return end_p, self.start_dir

    def get_plant_row(self, row_number, offset):
        start = self.start_p[row_number]
        c = self.bounded_gaussian.get(offset)

        cur_placement = start + self.start_dir * c
        placements = [cur_placement]
        while self.length - c > self.plant_params["plant_spacing_min"]:
            step = self.bounded_gaussian.get()
            cur_placement = cur_placement + self.start_dir * step
            placements.append(cur_placement)
            c += step

        # next offset
        dist = c - self.length
        return np.array(placements), dist

    def racing_line(self, row_number, spacing):
        super().racing_line(row_number, spacing)

        start = self.start_p[row_number] + 0.5 * (
            self.start_p[row_number + 1] - self.start_p[row_number]
        )
        c = 0

        dot = start
        line = [start]
        while c < self.length:
            dot += self.start_dir * spacing
            line.append(dot)
            c += spacing

        return np.array(line)

    def render(self):
        super().render()

        for x1, y1 in self.start_p:
            x2 = x1 + self.length * self.start_dir[0]
            y2 = y1 + self.length * self.start_dir[1]
            plt.plot([x1, x2], [y1, y2], color="g", linewidth=1)
