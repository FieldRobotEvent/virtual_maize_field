#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat May 15 05:13:19 2021

@author: fre
"""

import os
import rospkg
import csv
import numpy as np
from matplotlib import pyplot as plt


def plot_field(crop, weed_gt, weed_pred, litter_gt, litter_pred, marker_a, marker_b):
    plt.plot()
    plt.figure(figsize=(10, 10))
    plt.gca().axis("equal")
    labels = []

    # crops
    plt.scatter(crop[:, 0], crop[:, 1], color="g", marker=".")
    labels.append("crops")

    # weeds
    plt.scatter(
        weed_gt[:, 0],
        weed_gt[:, 1],
        color="r",
        marker=".",
        s=100,
        alpha=0.5,
    )
    labels.append("weeds_gt")

    plt.scatter(
        weed_pred[:, 0],
        weed_pred[:, 1],
        color="r",
        marker="*",
        s=100,
        alpha=0.5,
    )
    labels.append("weeds_pred")

    # litter
    plt.scatter(
        litter_gt[:, 0],
        litter_gt[:, 1],
        color="b",
        marker=".",
        s=100,
        alpha=0.5,
    )
    labels.append("litter_gt")

    plt.scatter(
        litter_pred[:, 0],
        litter_pred[:, 1],
        color="b",
        marker="*",
        s=100,
        alpha=0.5,
    )
    labels.append("litter_pred")

    # location markers
    plt.scatter(
        marker_a[:, 0], marker_a[:, 1], color="r", marker=".", alpha=0
    )  # just to extend the axis of the plot
    plt.text(
        marker_a[:, 0],
        marker_a[:, 1],
        "A",
        bbox={"facecolor": "red", "alpha": 0.5, "pad": 10},
        ha="center",
        va="center",
    )

    plt.scatter(
        marker_b[:, 0], marker_b[:, 1], color="r", marker=".", alpha=0
    )  # just to extend the axis of the plot
    plt.text(
        marker_b[:, 0],
        marker_b[:, 1],
        "B",
        bbox={"facecolor": "red", "alpha": 0.5, "pad": 10},
        ha="center",
        va="center",
    )

    plt.legend(labels)

    return plt


def read_dict(f_path):
    with open(f_path, "r") as f:
        reader = csv.reader(f)
        next(reader)  # skip header

        objects = {
            "location_marker_a": np.array([]).reshape(0, 2),
            "location_marker_b": np.array([]).reshape(0, 2),
            "crop": np.array([]).reshape(0, 2),
            "litter": np.array([]).reshape(0, 2),
            "weed": np.array([]).reshape(0, 2),
        }

        for line in reader:
            if line[2] in list(objects.keys()):
                xy = np.array([[float(line[0]), float(line[1])]])
                objects[line[2]] = np.concatenate((objects[line[2]], xy))

    return objects


if __name__ == "__main__":
    pkg_path = rospkg.RosPack().get_path("virtual_maize_field")

    gt_path = os.path.join(pkg_path, "map/map.csv")
    pred_path = os.path.join(pkg_path, "map/pred_map.csv")

    # read gt map to dict of arrays
    gt = read_dict(gt_path)
    # read pred map to dict of arrays
    pred = read_dict(pred_path)

    # compute closest points for weed
    # compute closest points for litters

    # compute scrore based on distance of the points

    # save mini_map
    fig = plot_field(crop=gt['crop'], weed_gt=gt['weed'], weed_pred=pred['weed'], litter_gt=gt['litter'], litter_pred=pred['litter'], marker_a=gt['location_marker_a'], marker_b=gt['location_marker_b'])
    fig_path = os.path.join(pkg_path, "map/evaluation_map.png")
    fig.savefig(fig_path, dpi=100)

    print("the detection errors are:")
    # print errors
    # print score
