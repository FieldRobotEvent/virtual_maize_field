from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

import numpy as np
from matplotlib import pyplot as plt

from virtual_maize_field.world_generator.utils import BoundedGaussian


class BaseSegment(ABC):
    def __init__(
        self,
        start_p: np.ndarray,
        start_dir: np.ndarray,
        plant_params: dict[str, Any],
        rng: np.random.Generator = np.random.default_rng(),
    ):
        self._rng = rng

        self.start_p = start_p
        self.start_dir = start_dir
        self.start_dir = self.start_dir / np.linalg.norm(self.start_dir)
        self.plant_params = plant_params
        self.bounded_gaussian = BoundedGaussian(
            self.plant_params["plant_spacing_min"],
            self.plant_params["plant_spacing_max"],
            rng=self._rng,
        )

    # Must return a tuple of the end points from the implemented segment
    # and the new direction for the next segment
    @abstractmethod
    def end(self) -> tuple[np.ndarray, ...]:
        raise NotImplementedError

    # returns a list of 2D coordinates for all plants in the segment and
    # a vector containing the offset to the next segment
    def placements(self, offset: np.ndarray | None = None):
        # Assuming the first plant should be roughly at the start of each row if no offset is given
        if offset == None:
            offset = np.full(
                (len(self.start_p)),
                -(
                    self.plant_params["plant_spacing_max"]
                    + self.plant_params["plant_spacing_min"]
                )
                / 2,
            )

        placements = []
        next_offset = []
        for i in range(len(self.start_p)):
            row, rest = self.get_plant_row(i, offset[i])
            placements.append(row)
            next_offset.append(rest)

        return placements, next_offset

    # Must return a list of 2D coordinates following the segments geometry
    # for the specified line and a distance left to the end point
    @abstractmethod
    def get_plant_row(self, row_number, offset):
        raise NotImplementedError

    # returns a list of all racing_lines from the segment
    def racing_lines(self, spacing):
        lines = []
        for i in range(len(self.start_p)):
            lines.append(self.racing_line(i, spacing))

        return lines

    # Must return a list of 2D coordinates following the segments geometry
    # for the specified line and a distance left to the end point
    @abstractmethod
    def racing_line(self, row_number, spacing):
        if row_number > len(self.start_p) - 1:
            raise ValueError(
                "There is no racing line " + row_number + "/" + len(self.start_p) - 1
            )

    # Must be implemented by the child class to render the segment
    @abstractmethod
    def render(self):
        x = self.start_p[:, 0]
        y = self.start_p[:, 1]
        plt.scatter(x, y, color="r", marker="o")

        x = self.end()[0][:, 0]
        y = self.end()[0][:, 1]
        plt.scatter(x, y, color="b", marker="x")
