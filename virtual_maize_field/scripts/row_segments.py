import numpy as np

from geometry import Geometry

class BaseSegment():
    def __init__(self, start_p, start_dir):
        self.start_p = start_p
        self.start_dir = start_dir
        self.start_dir = self.start_dir / np.linalg.norm(self.start_dir)

    def end(self):
        pass

    def dist(self):
        pass 

    def plant(self, offset):
        pass

class StraightSegment(BaseSegment):
    def __init__(self, start_p, start_dir, length):
        super(StraightSegment, self).__init__(start_p, start_dir)
        self.length = length
        self.distance = np.random.rand() * (length_max - length_min) + length_min

    def end(self):
        end_p = np.empty_like(self.start_p) 

        for i in range(len(self.start_p)):
            end_p[i] = self.start_p[i] + self.start_dir * self.distance

        return end_p

    def dist(self):
        pass 

    def plant(self, offset):
        pass

class CurveSegment(BaseSegment):
    def __init__(self, start_p, start_dir):
        super(CurveSegment, self).__init__(start_p, start_dir, )
        self.angle = 2 * np.pi * self.bounds.row_radius_min / self.row_segment_length_max * np.pi
        self.left = self.npr.choice([True, False])
        
        # calculate center 
        last_p = self.start_p[0] if self.left else self.start_p[-1]
        vec = self.start_p[0] - self.start_p[-1] if self.left else self.start_p[-1] - self.start_p[0]
        vec /= np.linalg.norm(vec)
        self.center = last_p + vec * self.bounds.row_radius_min

    def end(self):
        return Geometry.rotate(self.start_p, self.center, self.angle)

    def dist(self):
        pass

    def plant(self, offset):
        pass 