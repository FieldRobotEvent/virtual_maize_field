from __future__ import annotations

from typing import Any

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.patches import Arc

from virtual_maize_field.world_generator.utils import Geometry

from .base import BaseSegment


class CurvedSegment(BaseSegment):
    def __init__(
        self,
        start_p: np.ndarray,
        start_dir: np.ndarray,
        plant_params: dict[str, Any],
        radius: float,
        curve_dir: int,
        arc_measure: float,
        rng: np.random.Generator = np.random.default_rng(),
    ):
        super().__init__(start_p, start_dir, plant_params, rng=rng)

        self.radius = radius
        self.curve_dir = curve_dir
        self.arc_measure = arc_measure * (1 if self.curve_dir else -1)

        # calculate center
        last_p = self.start_p[0] if self.curve_dir else self.start_p[-1]
        vec = (
            self.start_p[0] - self.start_p[-1]
            if self.curve_dir
            else self.start_p[-1] - self.start_p[0]
        )
        vec /= np.linalg.norm(vec)
        self.center = last_p + vec * self.radius

    def end(self):
        end_p = Geometry.rotate(self.start_p, self.center, self.arc_measure)
        end_dir = Geometry.rotate(self.start_dir, [0, 0], self.arc_measure)
        return end_p, end_dir

    def get_plant_row(self, row_number, offset):
        start = self.start_p[row_number]
        r = np.linalg.norm(self.center - start)
        l = self.arc_measure * r
        rot = 1 if self.curve_dir else -1
        c = rot * self.bounded_gaussian.get(offset)

        cur_placement = Geometry.rotate(start, self.center, c / r)
        placements = [cur_placement]
        while (
            c < l - self.plant_params["plant_spacing_min"] or not self.curve_dir
        ) and (c > l + self.plant_params["plant_spacing_min"] or self.curve_dir):
            step = rot * self.bounded_gaussian.get(0.0)
            cur_placement = Geometry.rotate(cur_placement, self.center, step / r)
            placements.append(cur_placement)
            c += step

        # next offset
        dist = c - l if self.curve_dir else l - c
        return np.array(placements), dist

    def racing_line(self, row_number, spacing):
        super().racing_line(row_number, spacing)

        start = self.start_p[row_number] + 0.5 * (
            self.start_p[row_number + 1] - self.start_p[row_number]
        )
        r = np.linalg.norm(self.center - start)
        l = self.arc_measure * r
        rot = 1 if self.curve_dir else -1
        c = 0

        dot = start
        line = [start]
        while c < l:
            dot = Geometry.rotate(dot, self.center, rot * spacing / r)
            line.append(dot)
            c += spacing

        return np.array(line)

    def render(self):
        super().render()

        for x1, y1 in self.start_p:
            r = np.linalg.norm([x1, y1] - self.center)
            start_angle = -np.arctan2(self.start_dir[0], self.start_dir[1]) + (
                np.pi if not self.curve_dir else 0
            )
            end_angle = start_angle + self.arc_measure

            start_angle = np.rad2deg(start_angle)
            end_angle = np.rad2deg(end_angle)

            if start_angle > end_angle:
                start_angle, end_angle = end_angle, start_angle

            arc = Arc(
                self.center,
                2 * r,
                2 * r,
                0.0,
                start_angle,
                end_angle,
                color="g",
                linewidth=1,
            )
            plt.gca().add_patch(arc)
