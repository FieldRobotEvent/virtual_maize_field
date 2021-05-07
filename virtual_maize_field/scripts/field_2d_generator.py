import sys
import jinja2
import numpy as np
import cv2
import os
import rospkg
from matplotlib import pyplot as plt

from world_description import WorldDescription
from row_segments import StraightSegment, CurvedSegment, IslandSegment
from utils import BoundedGaussian


class Field2DGenerator:
    def __init__(self, world_description=WorldDescription()):
        self.wd = world_description
        np.random.seed(self.wd.structure["params"]["seed"])

    def render_matplotlib(self):
        # Segments
        for segment in self.segments:
            segment.render()

        # Plants
        plt.scatter(self.placements[:, 0], self.placements[:, 1], color="c", marker=".")

    def generate(self):
        self.chain_segments()
        self.center_plants()
        self.seed_weeds()
        self.generate_ground()
        self.fix_gazebo()
        self.render_to_template()
        return [self.sdf, self.heightmap]

    def chain_segments(self):
        # Generate start points
        rows = self.wd.rows_left + self.wd.rows_right
        x_start = (
            -self.wd.row_width / 2 - self.wd.row_width * (self.wd.rows_left - 1)
            if self.wd.rows_left > 0
            else self.wd.row_width / 2
        )
        x_end = (
            self.wd.row_width / 2 + self.wd.row_width * (self.wd.rows_right - 1)
            if self.wd.rows_right > 0
            else -self.wd.row_width / 2
        )
        current_p = np.array([np.linspace(x_start, x_end, rows), np.repeat(0, rows)]).T
        current_dir = [0, 1]

        # Placement parameters
        offset = None
        self.placements = [[] for _ in range(rows)]

        # Chain all segments from the world description
        self.segments = []
        for segment in self.wd.structure["segments"]:
            if segment["type"] == "straight":
                seg = StraightSegment(
                    current_p,
                    current_dir,
                    self.wd.structure["params"],
                    segment["length"],
                )
            elif segment["type"] == "curved":
                seg = CurvedSegment(
                    current_p,
                    current_dir,
                    self.wd.structure["params"],
                    segment["radius"],
                    segment["curve_dir"],
                    segment["arc_measure"],
                )
            elif segment["type"] == "island":
                seg = IslandSegment(
                    current_p,
                    current_dir,
                    self.wd.structure["params"],
                    segment["radius"],
                    segment["island_model"],
                    segment["island_model_radius"],
                    segment["island_row"],
                )
            else:
                raise ValueError("Unknown segment type. [" + segment["type"] + "]")

            # Collect all plant placements
            seg_placements, offset = seg.placements(offset)
            for row, seg_row in zip(self.placements, seg_placements):
                row.extend(seg_row)

            # Update current end points, direction and row length
            current_p, current_dir = seg.end()
            self.segments.append(seg)

        self.placements = np.vstack(self.placements)

        # Add bounden noise to placements
        bg = BoundedGaussian(
            -self.wd.structure["params"]["plant_placement_error_max"],
            self.wd.structure["params"]["plant_placement_error_max"],
        )

        # TODO There is a better way to do this
        new_placements = []
        for x, y in self.placements:
            x += bg.get()
            y += bg.get()
            new_placements.append([x, y])

        self.placements = np.array(new_placements)

    # Because the heightmap must be square and has to have a side length of 2^n + 1
    # this means that we could have smaller maps, by centering the placements around 0,0
    def center_plants(self):
        x_min = self.placements[:, 0].min()
        y_min = self.placements[:, 1].min()

        self.placements -= np.array([x_min, y_min])

        x_max = self.placements[:, 0].max()
        y_max = self.placements[:, 1].max()

        self.placements -= np.array([x_max, y_max]) / 2

    # The function calculates the placements of the weed plants and
    # stores them under self.weeds : np.array([[x,y],[x,y],...])
    def seed_weeds(self):
        # TODO Thijs
        # [ [x,y],[x,y],[x,y],... ]
        self.weeds = np.array([])

        # [ type1, type2, type3, ...]
        self.weeds_type = ["weed_01", "can", "ale"]

    def generate_ground(self):
        # Calculate image resolution
        metric_x_min = self.placements[:, 0].min()
        metric_x_max = self.placements[:, 0].max()
        metric_y_min = self.placements[:, 1].min()
        metric_y_max = self.placements[:, 1].max()

        metric_width = metric_x_max - metric_x_min + 4
        metric_height = metric_y_max - metric_y_min + 4

        resolution = 0.02
        min_image_size = int(
            np.ceil(max(metric_width / resolution, metric_height / resolution))
        )
        image_size = int(2 ** np.ceil(np.log2(min_image_size))) + 1

        # Generate noise
        heightmap = np.zeros((image_size, image_size))

        n = 0
        while 2 ** n < image_size:
            heightmap += (
                cv2.resize(
                    np.random.random((image_size // 2 ** n, image_size // 2 ** n)),
                    (image_size, image_size),
                )
                * (n + 1) ** 2
            )
            n += 1

        # Normalize heightmap
        heightmap -= heightmap.min()
        heightmap /= heightmap.max()

        offset = image_size // 2
        # Make plant placements flat and save the heights for the sdf renderer
        self.placements_ground_height = []
        for mx, my in self.placements:
            px = int(mx // resolution) + offset
            py = int(my // resolution) + offset

            height = heightmap[py, px]
            heightmap = cv2.circle(heightmap, (px, py), 2, height, -1)
            self.placements_ground_height.append(
                self.wd.structure["params"]["ground_max_elevation"] * height
            )

        # Convert to grayscale
        self.heightmap = (255 * heightmap[::-1, :]).astype(np.uint8)

        # Calc heightmap position
        self.metric_size = image_size * resolution
        self.heightmap_position = [
            metric_x_min - 2 + 0.5 * self.metric_size,
            metric_y_min - 2 + 0.5 * self.metric_size,
        ]

    def fix_gazebo(self):
        # move the plants to the center of the flat circles
        self.placements -= 0.01

        # set heightmap position to origin
        self.heightmap_position = [0, 0]

    def render_to_template(self):
        def into_dict(xy, ground_height, radius, height, mass, index):
            coordinate = dict()
            coordinate["type"] = np.random.choice(
                self.wd.structure["params"]["plant_types"].split(",")
            )
            inertia = dict()
            inertia["ixx"] = (mass * (3 * radius ** 2 + height ** 2)) / 12.0
            inertia["iyy"] = (mass * (3 * radius ** 2 + height ** 2)) / 12.0
            inertia["izz"] = (mass * radius ** 2) / 2.0
            coordinate["inertia"] = inertia
            coordinate["mass"] = mass
            coordinate["x"] = xy[0]
            coordinate["y"] = xy[1]
            coordinate["z"] = ground_height
            coordinate["radius"] = (
                radius
                + (2 * np.random.rand() - 1)
                * self.wd.structure["params"]["plant_radius_noise"]
            )
            if coordinate["type"] == "cylinder":
                coordinate["height"] = height
            coordinate["name"] = "{}_{:04d}".format(coordinate["type"], index)
            coordinate["yaw"] = np.random.rand() * 2.0 * np.pi
            return coordinate

        coordinates = [
            into_dict(
                plant,
                self.placements_ground_height[i],
                self.wd.structure["params"]["plant_radius"],
                self.wd.structure["params"]["plant_height_min"],
                self.wd.structure["params"]["plant_mass"],
                i,
            )
            for i, plant in enumerate(self.placements)
        ]

        pkg_path = rospkg.RosPack().get_path("virtual_maize_field")
        template_path = os.path.join(pkg_path, "scripts/field.world.template")
        template = open(template_path).read()
        template = jinja2.Template(template)
        self.sdf = template.render(
            coordinates=coordinates,
            seed=self.wd.structure["params"]["seed"],
            heightmap={
                "size": self.metric_size,
                "pos": {
                    "x": self.heightmap_position[0],
                    "y": self.heightmap_position[1],
                },
                "max_elevation": 0.2,
            },
        )
