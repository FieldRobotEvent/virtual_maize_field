import sys
import jinja2
import numpy as np
import os
import rospkg
from matplotlib import pyplot as plt

from world_description import WorldDescription
from row_segments import StraightSegment, CurvedSegment, IslandSegment


class Field2DGenerator:
    def __init__(self, world_description=WorldDescription()):
        self.wd = world_description
        np.random.seed(self.wd.structure["params"]["seed"])

    def generate(self):
        self.chainSegments()
        heightmap = self.generate_ground()
        sdf = self.render_to_template()
        return [sdf, heightmap]

    def chainSegments(self):
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
                    segment["iland_model"],
                    segment["iland_model_radius"],
                    segment["iland_row"],
                )
            else:
                raise ValueError("Unknown segment type. [" + segment_name + "]")

            seg_placements, offset = seg.placements(offset)
            for row, seg_row in zip(self.placements, seg_placements):
                row.extend(seg_row)
            # Update current end points, direction and row length
            current_p, current_dir = seg.end()
            self.segments.append(seg)

        self.placements = np.vstack(self.placements)
        self.placements += np.random.normal(
            scale=self.wd.structure["params"]["plant_radius_noise"],
            size=self.placements.shape,
        )

    def render_matplotlib(self):
        # Segments
        for segment in self.segments:
            segment.render()

        # Plants
        plt.scatter(self.placements[:, 0], self.placements[:, 1], color="c", marker=".")

    def generate_ground(self):
        # Generate heightmap png

        # Save in GAZEBO_MEDIA_PATH
        pass

    def render_to_template(self):
        def into_dict(xy, radius, height, mass, index):
            coordinate = dict()
            coordinate["type"] = np.random.choice(self.wd.structure['params']['plant_types'].split(','))
            inertia = dict()
            inertia["ixx"] = (mass * (3 * radius ** 2 + height ** 2)) / 12.0
            inertia["iyy"] = (mass * (3 * radius ** 2 + height ** 2)) / 12.0
            inertia["izz"] = (mass * radius ** 2) / 2.0
            coordinate["inertia"] = inertia
            coordinate["mass"] = mass
            coordinate["x"] = xy[0]
            coordinate["y"] = xy[1]
            coordinate["radius"] = (
                radius + (2 * np.random.rand() - 1) * self.wd.structure['params']['plant_radius_noise']
            )
            if coordinate["type"] == "cylinder":
                coordinate["height"] = height
            coordinate["name"] = "{}_{:04d}".format(coordinate["type"], index)
            coordinate["yaw"] = np.random.rand() * 2.0 * np.pi
            return coordinate

        coordinates = [
            into_dict(
                plant,
                self.wd.structure["params"]["plant_radius"],
                self.wd.structure["params"]["plant_height_min"],
                self.wd.structure["params"]["plant_mass"],
                i,
            )
            for i, plant in enumerate(self.placements)
        ]

        pkg_path = rospkg.RosPack().get_path('virtual_maize_field')
        template_path = os.path.join(pkg_path, "scripts/field.world.template")
        template = open(template_path).read()
        template = jinja2.Template(template)
        return template.render(coordinates=coordinates, seed=self.wd.structure['params']['seed'])


# class Field2DGenerator():
#     def __init__(self,
#         plant_radius = .30,
#         row_width = 0.75,
#         plant_offset = .50,
#         max_angle_variation = 0.15,
#         num_plant_pairs = 20,
#         num_rows_left = 2,
#         num_rows_right = 2,
#         plant_height = 0.3,
#         plant_mass = 5.0,
#         radius_noise_range = 0.05,
#         position_div = .03,
#         dropout = 0.0,
#         seed = None,
#         types=",".join(["maize_01", "maize_02"])):

#         if seed is None:
#             seed = int(datetime.now().timestamp()*1000)%8192

#         for k,v in locals().items():
#             self.__setattr__(k,v)

#         np.random.seed(self.seed)
#         self.reset()

#     def reset(self):
#         position = np.matrix([[0],[0],[1]])
#         self.way_points = [position]
#         self.world_T_local = np.identity(3)
#         angles = []
#         self.placements = []

#         for _ in range(self.num_plant_pairs):
#             angle_variation = ((np.random.rand() * 2) - 1) * self.max_angle_variation
#             angles.append(angle_variation)

#             tx = self.plant_offset
#             ty = 0

#             T = np.matrix([
#                 [1, 0, tx],
#                 [0, 1, ty],
#                 [0, 0, 1]
#             ])

#             R = np.matrix([
#                 [ np.cos(angle_variation), np.sin(angle_variation), 0],
#                 [-np.sin(angle_variation), np.cos(angle_variation), 0],
#                 [                       0,                       0, 1]
#             ])

#             self.world_T_local = self.world_T_local * T * R
#             position = self.world_T_local * np.matrix([[0],[0],[1]])
#             self.way_points.append(position)

#             for i in range(self.num_rows_left):
#                 width = self.row_width * (i + 0.5)
#                 placement_l = self.world_T_local * np.matrix([[0],[ width],[1]])
#                 self.placements.append(placement_l)

#             for i in range(self.num_rows_right):
#                 width = self.row_width * (i + 0.5)
#                 placement_r = self.world_T_local * np.matrix([[0],[-width],[1]])
#                 self.placements.append(placement_r)

#         self.placements = np.array(np.stack([p.T for p in self.placements])).T
#         self.placements += np.random.normal(scale=self.position_div, size=self.placements.shape)
#         self.placements = self.placements[:,self.dropout<=np.random.rand(self.placements.shape[1])]

#     def get_distance_to_closest_pumpkin(self, x, y, theta):
#         s = np.sin(theta)
#         c = np.cos(theta)
#         LOCAL_T_WORLD = np.matrix([
#             [c, -s, (-x*np.cos(2*theta) - x + y*np.sin(2*theta))/(2*c)],
#             [s,  c,                                         -x*s - y*c],
#             [0,  0,                                                  1]])

#         LOCAL_placements = LOCAL_T_WORLD * self.placements
#         dist_placement = np.linalg.norm(LOCAL_placements[0:2], axis=0).min() - self.plant_radius
#         return dist_placement

#     def render_to_template(self, template):
#         def into_dict(xy, radius, height, mass, index):
#             types = []
#             for t in self.types.split(","):
#                 if t in AVAILABLE_TYPES:
#                     types.append(t)
#                 else:
#                     print("Error: model type", t, "is not available.")
#             coordinate = dict()
#             coordinate["type"] = np.random.choice(types)
#             inertia = dict()
#             inertia["ixx"] = (mass * (3 * radius**2 + height**2)) / 12.
#             inertia["iyy"] = (mass * (3 * radius**2 + height**2)) / 12.
#             inertia["izz"] = (mass * radius**2) / 2.
#             coordinate["inertia"] = inertia
#             coordinate["mass"] = mass
#             coordinate["x"] = xy[0]
#             coordinate["y"] = xy[1]
#             coordinate["radius"] = radius + (2*np.random.rand() - 1)*self.radius_noise_range
#             if coordinate["type"] == "cylinder":
#                 coordinate["height"] = height
#             coordinate["name"] = "{}_{:04d}".format(coordinate["type"], index)
#             coordinate["yaw"] = np.random.rand() * 2.0 * np.pi
#             return coordinate
#         coordinates = [into_dict(row,self.plant_radius, self.plant_height, self.plant_mass, i) for i, row in enumerate(self.placements.T[:,:-1])]
#         template = jinja2.Template(template)
#         return template.render(coordinates=coordinates, seed=self.seed)
