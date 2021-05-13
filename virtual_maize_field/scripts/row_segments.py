import numpy as np
from abc import ABC, abstractmethod

# rendering
from matplotlib import pyplot as plt
from matplotlib.patches import Arc

from utils import Geometry, BoundedGaussian


class BaseSegment(ABC):
    def __init__(self, start_p, start_dir, plant_params):
        self.start_p = start_p
        self.start_dir = start_dir
        self.start_dir = self.start_dir / np.linalg.norm(self.start_dir)
        self.plant_params = plant_params
        self.bounded_gaussian = BoundedGaussian(
            self.plant_params["plant_spacing_min"], self.plant_params["plant_spacing_max"]
        )

    # Must return a touple of the end points from the implemented segment
    # and the new direction for the next segment
    @abstractmethod
    def end(self):
        raise NotImplementedError

    # returns a list of 2D coordinates for all plants in the segment and
    # a vector containing the offset to the next segment
    def placements(self, offset=None):
        # Assuming the first plant should be roughly at the start of each row if no offset is given
        if offset == None:
            offset = np.full(
                (len(self.start_p)),
                -(self.plant_params["plant_spacing_max"] + self.plant_params["plant_spacing_min"])
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
        raise NotADirectoryError

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
            raise ValueError("There is no racing line " + row_number + "/" + len(self.start_p) - 1)

    # Must be implemented by the child class to render the segment
    @abstractmethod
    def render(self):
        x = self.start_p[:, 0]
        y = self.start_p[:, 1]
        plt.scatter(x, y, color="r", marker="o")

        x = self.end()[0][:, 0]
        y = self.end()[0][:, 1]
        plt.scatter(x, y, color="b", marker="x")


class StraightSegment(BaseSegment):
    def __init__(self, start_p, start_dir, plant_params, length):
        super(StraightSegment, self).__init__(start_p, start_dir, plant_params)
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
        super(StraightSegment, self).racing_line(row_number, spacing)

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
        super(StraightSegment, self).render()

        for x1, y1 in self.start_p:
            x2 = x1 + self.length * self.start_dir[0]
            y2 = y1 + self.length * self.start_dir[1]
            plt.plot([x1, x2], [y1, y2], color="g", linewidth=1)


class CurvedSegment(BaseSegment):
    def __init__(self, start_p, start_dir, plant_params, radius, curve_dir, arc_measure):
        super(CurvedSegment, self).__init__(start_p, start_dir, plant_params)
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
        while (c < l - self.plant_params["plant_spacing_min"] or not self.curve_dir) and (
            c > l + self.plant_params["plant_spacing_min"] or self.curve_dir
        ):
            step = rot * self.bounded_gaussian.get(0.0)
            cur_placement = Geometry.rotate(cur_placement, self.center, step / r)
            placements.append(cur_placement)
            c += step

        # next offset
        dist = c - l if self.curve_dir else l - c
        return np.array(placements), dist

    def racing_line(self, row_number, spacing):
        super(CurvedSegment, self).racing_line(row_number, spacing)

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
        super(CurvedSegment, self).render()

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
                self.center, 2 * r, 2 * r, 0.0, start_angle, end_angle, color="g", linewidth=1
            )
            plt.gca().add_patch(arc)


class IslandSegment(BaseSegment):
    def __init__(
        self,
        start_p,
        start_dir,
        plant_params,
        radius,
        island_model,
        island_model_radius,
        island_row,
    ):
        super(IslandSegment, self).__init__(start_p, start_dir, plant_params)
        self.radius = radius
        self.island_model = island_model
        self.island_model_radius = island_model_radius
        self.island_row_radius = island_model_radius if island_model_radius > radius else radius
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
            self.radius_left = d_l - np.linalg.norm(self.start_p_left[0] - self.start_p_left[-1])
            self.radius_right = d_l - np.linalg.norm(self.start_p_right[0] - self.start_p_right[-1])
            self.length = 2 * d_l * np.tan(self.angle)
        else:
            self.angle = a_r
            self.radius_left = d_r - np.linalg.norm(self.start_p_left[0] - self.start_p_left[-1])
            self.radius_right = d_r - np.linalg.norm(self.start_p_right[0] - self.start_p_right[-1])
            self.length = 2 * d_r * np.tan(self.angle)

        # Island
        self.center_island = self.start_p[self.island_row] + self.start_dir * 0.5 * self.length

        # Left
        self.entrence_left = CurvedSegment(
            self.start_p_left, self.start_dir, self.plant_params, self.radius_left, True, self.angle
        )
        lp1, ld1 = self.entrence_left.end()
        self.middle_left = CurvedSegment(
            lp1, ld1, plant_params, self.island_row_radius, False, 2 * self.angle
        )
        lp2, ld2 = self.middle_left.end()
        self.exit_left = CurvedSegment(lp2, ld2, plant_params, self.radius_left, True, self.angle)

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
            p1, off1 = self.entrence_right.get_plant_row(row_number - self.island_row, offset)
            p2, off2 = self.middle_right.get_plant_row(row_number - self.island_row, off1)
            p3, next_offset = self.exit_right.get_plant_row(row_number - self.island_row, off2)
            placements = np.vstack([p1, p2, p3])
        else:
            pel, off1 = self.entrence_left.get_plant_row(row_number, offset)
            pml, off2 = self.middle_left.get_plant_row(row_number, off1)
            pxl, nol = self.exit_left.get_plant_row(row_number, off2)

            per, off1 = self.entrence_right.get_plant_row(row_number - self.island_row, offset)
            pmr, off2 = self.middle_right.get_plant_row(row_number - self.island_row, off1)
            pxr, nor = self.exit_right.get_plant_row(row_number - self.island_row, off2)

            placements = np.vstack([pel, per, pml, pmr, pxl, pxr])
            next_offset = min(nol, nor)

        return placements, next_offset

    def racing_line(self, row_number, spacing):
        raise NotImplementedError

    def render(self):
        super(IslandSegment, self).render()

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
