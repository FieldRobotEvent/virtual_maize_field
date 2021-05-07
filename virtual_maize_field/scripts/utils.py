import numpy as np


class Geometry:
    @staticmethod
    def rotate(p, origin=(0, 0), angle=0):
        R = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        o = np.atleast_2d(origin)
        p = np.atleast_2d(p)
        return np.around(np.squeeze((R @ (p.T - o.T) + o.T).T), decimals=5)


class BoundedGaussian:
    def __init__(self, lower, upper, span=1):
        self.lower = lower
        self.upper = upper
        self.span = span

    def get(self, offset=0.0):
        mean = (self.upper + self.lower) / 2
        sigma = self.upper - mean
        while True:
            num = np.random.normal(mean, sigma / self.span)
            if num > self.lower and num < self.upper:
                return num + offset
