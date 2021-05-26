#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospkg
import csv
import numpy as np
from matplotlib import pyplot as plt
import itertools
import argparse


def point_score(a, b):
    # score for detections of task 3 as defined by: https://www.fieldrobot.com/event/index.php/contest/task-3/
    # x: distance error or Euclidean distance	points
    # x ≤ 2cm	10
    # 2 cm < x ≤ 37.5 cm	10.56 - 0.2817 * x
    # x > 37.5 cm	0

    dist = np.sqrt(np.square(a[0] - b[0]) + np.square(a[1] - b[1]))
    if dist <= 0.02:
        score = 15
    elif dist <= 0.375:
        score = max(0, (15.56 - 28.17 * dist))
    else:
        score = -5

    return score, dist


def compute_score(gt, pred):
    best_score = 0
    best_gt_matches = []
    best_pred_matches = []
    best_dist = []

    # make gt and pred equally long, by adding additional samples far outside the field
    # only use the permute of on of the arrays.
    # or remove duplicates.

    # TODO make this more efficient, as the timecomplexity is now quadratic
    # comparing all possible orders of gt objects and pred objects to maximize the score
    for g in itertools.permutations(gt):
        for p in itertools.permutations(pred):
            score = 0
            gt_matches = []
            pred_matches = []
            dist = []
            for i in range(min(len(g), len(p))):
                curr_score, curr_dist = point_score(g[i], p[i])
                score += curr_score
                if curr_score > 0:
                    gt_matches.append(g[i])
                    pred_matches.append(p[i])
                    dist.append(curr_dist)

            # add penalty for additional fp
            if len(p) > len(g):
                fp = len(p) - len(g)
                score -= fp * 5

            # TODO there is currently no penalty for false positives
            if score > best_score:
                best_score = score
                best_gt_matches = gt_matches
                best_pred_matches = pred_matches
                best_dist = dist

    return best_score, np.array(best_gt_matches), np.array(best_pred_matches), np.array(best_dist)


def plot_field(
    crop,
    weed_gt,
    weed_pred,
    litter_gt,
    litter_pred,
    matches,
    dist,
    marker_a,
    marker_b,
    weed_score,
    litter_score,
):
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

    # draw matches
    for i in range(len(matches[0])):
        plt.plot(
            [matches[0][i][0], matches[1][i][0]],
            [matches[0][i][1], matches[1][i][1]],
            linewidth=2,
            markersize=12,
            color="k",
        )

        plt.text(matches[0][i][0] + 0.05, matches[0][i][1] + 0.05, "%.2f" % dist[i])

    # draw score
    txt = "Weed score: %.2f \nLitter score: %.2f \nTotal score: %.2f" % (
        weed_score,
        litter_score,
        weed_score + litter_score,
    )
    plt.figtext(
        0.5,
        0.01,
        txt,
        ha="center",
        fontsize=10,
        bbox={"facecolor": "orange", "alpha": 0.5, "pad": 5},
    )

    plt.grid()
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

        try:
            for line in reader:
                if line[2] in list(objects.keys()):
                    xy = np.array([[float(line[0]), float(line[1])]])
                    objects[line[2]] = np.concatenate((objects[line[2]], xy))
        except:
            print("Something is wrong with %s" % f_path)
            print("Probably there is an empty line in your file")

    return objects


if __name__ == "__main__":
    pkg_path = rospkg.RosPack().get_path("virtual_maize_field")

    parser = argparse.ArgumentParser(
        description="Comapre gt_detection map with a prediiction_detection map"
    )
    parser.add_argument(
        "--gt_path",
        type=str,
        default=os.path.join(pkg_path, "map/map.csv"),
        help="Path of the ground truth csv file generated by the map generator",
    )
    parser.add_argument(
        "--pred_path",
        type=str,
        default=os.path.join(pkg_path, "map/example_pred_map.csv"),
        help="Path of the prediction csv file, generated by the competitor performing the mapping task",
    )
    parser.add_argument(
        "--out_map",
        type=str,
        default=os.path.join(pkg_path, "map/evaluation_map.png"),
        help="Output location for a visual map",
    )
    parser.add_argument("--out_dpi", type=int, default=1000, help="DPI of the output map")

    args = parser.parse_args()

    # read gt and map to dict of arrays
    gt = read_dict(args.gt_path)
    pred = read_dict(args.pred_path)

    weed_score, weed_gt_matches, weed_pred_matches, weed_dist = compute_score(
        gt["weed"], pred["weed"]
    )
    litter_score, litter_gt_matches, litter_pred_matches, litter_dist = compute_score(
        gt["litter"], pred["litter"]
    )

    gt_matches = np.concatenate((weed_gt_matches, litter_gt_matches))
    pred_matches = np.concatenate((weed_pred_matches, litter_pred_matches))
    dist = np.concatenate((weed_dist, litter_dist))

    # save map
    fig = plot_field(
        crop=gt["crop"],
        weed_gt=gt["weed"],
        weed_pred=pred["weed"],
        litter_gt=gt["litter"],
        litter_pred=pred["litter"],
        matches=[gt_matches, pred_matches],
        dist=dist,
        marker_a=gt["location_marker_a"],
        marker_b=gt["location_marker_b"],
        weed_score=weed_score,
        litter_score=litter_score,
    )
    fig.savefig(args.out_map, dpi=args.out_dpi)

    print("Weed score: %.2f" % weed_score)
    print("Litter score: %.2f" % litter_score)
    print("Total score: %.2f" % (weed_score + litter_score))
