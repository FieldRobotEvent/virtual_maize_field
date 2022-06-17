from __future__ import annotations

from typing import Any

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.patches import Arc

from .base import BaseSegment
from .curve import CurvedSegment


class IslandSegment(BaseSegment):
    def __init__(
        self,
        start_p: np.ndarray,
        start_dir: np.ndarray,
        plant_params: dict[str, Any],
        radius: float,
        island_model: str,
        island_model_radius: float,
        island_row: int,
        rng: np.random.Generator = np.random.default_rng(),
    ):
        super().__init__(start_p, start_dir, plant_params, rng=rng)

        self.radius = radius
        self.island_model = island_model
        self.island_model_radius = island_model_radius
        self.island_row_radius = (
            island_model_radius if island_model_radius > radius else radius
        )
        self.island_row = island_row

        # start p vec
        vec = self.start_p[0] - self.start_p[-1]
        vec /= np.linalg.norm(vec)

        # split row
        self.start_p_left = self.start_p[: self.island_row + 1]
        self.start_p_right = self.start_p[self.island_row :]

        # start circles
        center_start_left = self.start_p[0] + vec * self.radius
        d_l = np.linalg.norm(start_p[self.island_row] - center_start_left)
        a_l = np.arccos(d_l / (d_l + self.island_row_radius))

        center_start_right = self.start_p[-1] - vec * self.radius
        d_r = np.linalg.norm(start_p[self.island_row] - center_start_right)
        a_r = np.arccos(d_r / (d_r + self.island_row_radius))

        # check which is bigger and us it
        if d_l > d_r:
            self.angle = a_l
            self.radius_left = d_l - np.linalg.norm(
                self.start_p_left[0] - self.start_p_left[-1]
            )
            self.radius_right = d_l - np.linalg.norm(
                self.start_p_right[0] - self.start_p_right[-1]
            )
            self.length = 2 * d_l * np.tan(self.angle)
        else:
            self.angle = a_r
            self.radius_left = d_r - np.linalg.norm(
                self.start_p_left[0] - self.start_p_left[-1]
            )
            self.radius_right = d_r - np.linalg.norm(
                self.start_p_right[0] - self.start_p_right[-1]
            )
            self.length = 2 * d_r * np.tan(self.angle)

        # Island
        self.center_island = (
            self.start_p[self.island_row] + self.start_dir * 0.5 * self.length
        )

        # Left
        self.entrence_left = CurvedSegment(
            self.start_p_left,
            self.start_dir,
            self.plant_params,
            self.radius_left,
            True,
            self.angle,
        )
        lp1, ld1 = self.entrence_left.end()
        self.middle_left = CurvedSegment(
            lp1, ld1, plant_params, self.island_row_radius, False, 2 * self.angle
        )
        lp2, ld2 = self.middle_left.end()
        self.exit_left = CurvedSegment(
            lp2, ld2, plant_params, self.radius_left, True, self.angle
        )

        # Right
        self.entrence_right = CurvedSegment(
            self.start_p_right,
            self.start_dir,
            self.plant_params,
            self.radius_right,
            False,
            self.angle,
        )
        rp1, rd1 = self.entrence_right.end()
        self.middle_right = CurvedSegment(
            rp1, rd1, plant_params, self.island_row_radius, True, 2 * self.angle
        )
        rp2, rd2 = self.middle_right.end()
        self.exit_right = CurvedSegment(
            rp2, rd2, plant_params, self.radius_right, False, self.angle
        )

    def end(self):
        end_p = np.empty_like(self.start_p)

        end_p[: self.island_row + 1], _ = self.exit_left.end()
        end_p[self.island_row :], _ = self.exit_right.end()

        return end_p, self.start_dir

    def get_plant_row(self, row_number, offset):
        if row_number < self.island_row:
            p1, off1 = self.entrence_left.get_plant_row(row_number, offset)
            p2, off2 = self.middle_left.get_plant_row(row_number, off1)
            p3, next_offset = self.exit_left.get_plant_row(row_number, off2)
            placements = np.vstack([p1, p2, p3])
        elif row_number > self.island_row:
            p1, off1 = self.entrence_right.get_plant_row(
                row_number - self.island_row, offset
            )
            p2, off2 = self.middle_right.get_plant_row(
                row_number - self.island_row, off1
            )
            p3, next_offset = self.exit_right.get_plant_row(
                row_number - self.island_row, off2
            )
            placements = np.vstack([p1, p2, p3])
        else:
            pel, off1 = self.entrence_left.get_plant_row(row_number, offset)
            pml, off2 = self.middle_left.get_plant_row(row_number, off1)
            pxl, nol = self.exit_left.get_plant_row(row_number, off2)

            per, off1 = self.entrence_right.get_plant_row(
                row_number - self.island_row, offset
            )
            pmr, off2 = self.middle_right.get_plant_row(
                row_number - self.island_row, off1
            )
            pxr, nor = self.exit_right.get_plant_row(row_number - self.island_row, off2)

            placements = np.vstack([pel, per, pml, pmr, pxl, pxr])
            next_offset = min(nol, nor)

        return placements, next_offset

    def racing_line(self, row_number, spacing):
        raise NotImplementedError

    def render(self):
        super().render()

        # center clearance
        arc = Arc(
            self.center_island,
            2 * self.island_row_radius,
            2 * self.island_row_radius,
            0.0,
            0,
            360,
            color="b",
            linewidth=1,
        )
        plt.gca().add_patch(arc)

        # island center
        plt.scatter(self.center_island[0], self.center_island[1], color="b", marker=".")

        self.entrence_left.render()
        self.middle_left.render()
        self.exit_left.render()

        self.entrence_right.render()
        self.middle_right.render()
        self.exit_right.render()
