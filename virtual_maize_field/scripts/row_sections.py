import numpy as np

from geometry import Geometry

class BaseSection():
    def __init__(self, start_p, start_dir):
        for k,v in locals().items():
            self.__setattr__(k,v)
        self.start_dir = self.start_dir / np.linalg.norm(self.start_dir)

    def end(self):
        pass

    def plant(self, offset):
        pass

class StraightSection(BaseSection):
    def __init__(self, start_p, start_dir, distance):
        super(StraightSection, self).__init__(start_p, start_dir)
        self.distance = distance

    def end(self):
        end_p = np.empty_like(self.start_p) 

        for i in range(len(self.start_p)):
            end_p[i] = self.start_p[i] + self.start_dir * self.distance

        return end_p

    def plant(self, offset):
        pass

class CurveSection(BaseSection):
    def __init__(self, start_p, start_dir, angle, min_radius, left=True):
        super(CurveSection, self).__init__(start_p, start_dir)
        self.angle = angle
        self.min_radius = min_radius
        self.left = left
        
        # calculate center 
        last_p = self.start_p[0] if self.left else self.start_p[-1]
        vec = self.start_p[0] - self.start_p[-1] if self.left else self.start_p[-1] - self.start_p[0]
        vec /= np.linalg.norm(vec)
        self.center = last_p + vec * self.min_radius

    def end(self):
        return Geometry.rotate(self.start_p, self.center, self.angle)