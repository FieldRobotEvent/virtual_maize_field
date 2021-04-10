import numpy as np

class Geometry():
    @staticmethod
    def rotate(p, origin=(0, 0), angle=0):
        R = np.array([[np.cos(angle), -np.sin(angle)],
                    [np.sin(angle),  np.cos(angle)]])
        o = np.atleast_2d(origin)
        p = np.atleast_2d(p)
        return np.squeeze((R @ (p.T-o.T) + o.T).T)

    @staticmethod
    def getTangentPoints(p, center):
        return Geometry.rotate(p, origin=center, angle=-np.pi/2)