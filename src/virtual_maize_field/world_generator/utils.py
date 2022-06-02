from __future__ import annotations

from argparse import ArgumentParser
from inspect import getfullargspec

import numpy as np


class Geometry:
    @staticmethod
    def rotate(p, origin=(0, 0), angle=0):
        R = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        o = np.atleast_2d(origin)
        p = np.atleast_2d(p)
        return np.around(np.squeeze((R @ (p.T - o.T) + o.T).T), decimals=5)


class BoundedGaussian:
    def __init__(
        self, lower, upper, span=1, rng: np.random.Generator = np.random.default_rng()
    ):
        self.lower = lower
        self.upper = upper
        self.span = span

        self._rng = rng

    def get(self, offset=0.0):
        mean = (self.upper + self.lower) / 2
        sigma = self.upper - mean
        while True:
            num = self._rng.normal(mean, sigma / self.span)
            if num > self.lower and num < self.upper:
                return num + offset


def parser_from_function(func: object, description: str = "") -> ArgumentParser:
    # Dynamically create argument parser with world description parameters
    argspec = getfullargspec(func)
    possible_kwargs = argspec.args[1:]
    defaults = argspec.defaults

    parser = ArgumentParser(
        description=description,
    )
    for argname, default in zip(possible_kwargs, defaults):
        # we analyze the default value's type to guess the type for that argument
        if type(default) == list:
            parser.add_argument(
                "--" + argname,
                type=type(default[0]),
                nargs="*",
                help=f"default_value: {default}",
                required=False,
            )
        else:
            parser.add_argument(
                "--" + argname,
                type=type(default),
                help=f"default_value: {default}",
                required=False,
            )

    return parser
