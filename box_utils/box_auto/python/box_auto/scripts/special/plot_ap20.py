#!/usr/bin/env python3

import argparse
import os
import matplotlib.pyplot as plt
import numpy as np
import rosbag
from pathlib import Path


def extract_pose_from_bag(bag_file, topic_name):
    """
    Extracts x, y coordinates from a PoseStamped topic in a bag file.
    """
    x_coords = []
    y_coords = []

    with rosbag.Bag(bag_file, "r") as bag:
        for _topic, msg, _t in bag.read_messages(topics=[topic_name]):
            x_coords.append(msg.point.x)
            y_coords.append(msg.point.y)

    return np.array(x_coords), np.array(y_coords)


def plot_trajectory(x_coords, y_coords, output_file, tag):
    """
    Plots the x, y trajectory.
    """
    # Calculate data ranges
    x_min, x_max = np.min(x_coords), np.max(x_coords)
    y_min, y_max = np.min(y_coords), np.max(y_coords)
    x_range = x_max - x_min
    y_range = y_max - y_min
    aspect_ratio = y_range / x_range

    # Set figure dimensions
    width = 10
    height = width * aspect_ratio
    fig, ax = plt.subplots(figsize=(width, height))

    # Plot trajectory
    ax.plot(x_coords, y_coords, "-", color="blue", label="Trajectory AP20", linewidth=2)

    # Style settings
    ax.set_xlabel("X Coordinate [m]", fontsize=12)
    ax.set_ylabel("Y Coordinate [m]", fontsize=12)
    ax.set_title(f"Trajectory from {tag}", fontsize=14, pad=20)
    ax.legend(fontsize=10)

    # Add padding and round to the nearest 10m for grid alignment
    padding = 10
    x_min_grid = np.floor((x_min - padding) / 10.0) * 10
    x_max_grid = np.ceil((x_max + padding) / 10.0) * 10
    y_min_grid = np.floor((y_min - padding) / 10.0) * 10
    y_max_grid = np.ceil((y_max + padding) / 10.0) * 10

    # Set grid lines every 10 meters
    ax.set_xticks(np.arange(x_min_grid, x_max_grid + 1, 10))
    ax.set_yticks(np.arange(y_min_grid, y_max_grid + 1, 10))
    ax.grid(True, which="major", alpha=0.3)

    # Set equal aspect ratio
    ax.set_aspect("equal")

    # Add padding to the plot limits
    padding_ratio = 0.05
    x_padding = x_range * padding_ratio
    y_padding = y_range * padding_ratio
    ax.set_xlim(x_min - x_padding, x_max + x_padding)
    ax.set_ylim(y_min - y_padding, y_max + y_padding)

    # Style spines
    for spine in ax.spines.values():
        spine.set_color("gray")
        spine.set_linewidth(0.5)

    # Adjust layout and save the plot
    plt.tight_layout()
    plt.savefig(output_file)
    print(f"Plot saved to {output_file}")


def main():
    parser = argparse.ArgumentParser(description="Plot x, y trajectory from a PoseStamped topic in a bag file.")
    parser.add_argument("--bag_file", type=str, help="Path to the ROS bag file.")
    parser.add_argument(
        "--topic_name", default="/gt_box/ap20/prism_position", type=str, help="Name of the PoseStamped topic."
    )
    parser.add_argument("--output_folder", type=str, default="./", help="Path to save the plot.")
    args = parser.parse_args()

    # Validate inputs
    if not os.path.isfile(args.bag_file):
        raise FileNotFoundError(f"Bag file not found: {args.bag_file}")

    # Extract data and plot
    x_coords, y_coords = extract_pose_from_bag(args.bag_file, args.topic_name)
    if len(x_coords) == 0:
        raise ValueError(f"No data found in topic {args.topic_name}")

    tag = str(Path(args.bag_file).stem).replace("jetson_ap20", "prism_trajectory")
    out_file = os.path.join(args.output_folder, tag + ".png")
    plot_trajectory(x_coords, y_coords, out_file, tag)


if __name__ == "__main__":
    main()
