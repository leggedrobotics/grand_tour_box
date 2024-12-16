# Example usage
# USAGE: python3 ~/git/grand_tour_box/box_utils/box_auto/scripts/topic_freq.py --folder /mission_data

import rosbag
import matplotlib

matplotlib.use("Agg")  # Set the backend to Agg for headless environments
import matplotlib.pyplot as plt
import numpy as np
import os
import pathlib
from pathlib import Path
import subprocess
from matplotlib.ticker import ScalarFormatter

MISSION_DATA = os.environ.get("MISSION_DATA", "/mission_data")


def run_rosbag_command(bag_file, output_file):
    # Define the command to run
    command = ["rosbag", "info", "--freq", bag_file]

    try:
        # Run the command and capture the output
        result = subprocess.run(command, capture_output=True, text=True, check=True)

        with open(output_file, "a") as file:
            file.write(result.stdout)

        print(f"Output successfully written to {output_file}")

    except subprocess.CalledProcessError as e:
        # Handle error if the command fails
        print(f"Error running command: {e}")
        print(f"Error output: {e.stderr}")


def read_rosbag_and_generate_histograms(rosbag_path, output_dir, name, skip_same_timestamps=True):
    # Ensure the output directory exists
    os.makedirs(output_dir, exist_ok=True)

    # Open the ROS bag file
    bag = rosbag.Bag(rosbag_path, "r")

    # Dictionary to hold timestamps for each topic
    topic_timestamps = {}

    topic_number_stame_timestamps = {}

    # Iterate over all messages in the bag
    for topic, msg, _t in bag.read_messages():
        try:
            # Extract the timestamp from the message header
            stamp = msg.header.stamp
            # Convert ROS Time to seconds (secs + nsecs to float)
            time_in_sec = stamp.secs + stamp.nsecs * 1e-9
            # Append the timestamp to the respective topic list in the dictionary
            if topic not in topic_timestamps:
                topic_timestamps[topic] = []
                topic_number_stame_timestamps[topic] = 0

            if skip_same_timestamps:
                if len(topic_timestamps[topic]) > 0 and topic_timestamps[topic][-1] == time_in_sec:
                    topic_number_stame_timestamps[topic] += 1
                    print(f"   Topic: {topic} has a repeat timestamp at {time_in_sec}")
                    continue
            topic_timestamps[topic].append(time_in_sec)
        except AttributeError:
            # If the message does not have a header with a timestamp, skip it
            continue

    # Close the bag file
    bag.close()

    print("\n\n", rosbag_path)
    # Number of unique topics
    num_topics = len(topic_timestamps.keys())
    if num_topics == 0:
        print("   No topics with valid timestamps found.")
        return

    output_txt_file = os.path.join(output_dir, "freq.txt")
    for k, v in topic_number_stame_timestamps.items():
        if v > 0:
            msg = f"   Topic: {k} has {v} same timestamps"
            print(msg)
            with open(output_txt_file, "a") as file:
                file.write(f"\n{rosbag_path}: {msg}\n")

    # Create subplots - determine grid size
    cols = 3  # Number of columns for subplots
    rows = (num_topics + cols - 1) // cols  # Calculate rows needed for all topics

    fig, axs = plt.subplots(rows, cols, figsize=(20, 5 * rows))
    axs = axs.flatten()  # Flatten the array of subplots for easy iteration

    # This is  the colormap I'd like to use.
    cm = plt.cm.get_cmap("inferno_r")

    for i, (topic, timestamps) in enumerate(topic_timestamps.items()):
        if len(timestamps) < 2:
            print(f"   Topic: {topic} has not enough data points a histogram.")
            continue

        # Compute deltas between consecutive timestamps
        deltas = np.diff(timestamps).clip(min=0.001)[1:-1]  # Clip to prevent division by zero

        # Plot histogram of time deltas in the appropriate subplot
        n, bins, patches = axs[i].hist(1 / deltas, bins=100, alpha=0.9, color="blue")
        bin_centers = 0.5 * (bins[:-1] + bins[1:])

        # scale values to interval [0,1]
        col = bin_centers - min(bin_centers)
        col /= max(col)

        for c, p in zip(col, patches):
            plt.setp(p, "facecolor", cm(c * 0.6 + 0.2))

        same_ts_msg = (
            ""
            if topic_number_stame_timestamps[topic] == 0
            else f"{topic_number_stame_timestamps[topic]} frames with repeat timestamps"
        )
        axs[i].text(
            0.3,
            0.9,
            same_ts_msg,
            transform=axs[i].transAxes,
            fontsize=12,
            color="red",
            ha="center",
            va="center",
            alpha=0.5,
            bbox=dict(facecolor="white", alpha=0.8),
        )

        axs[i].set_title(f"Histogram for Topic: {topic}")
        axs[i].set_xlabel("Delta Time (Hz)")
        axs[i].set_ylabel("Frequency")
        axs[i].grid(True)

        axs[i].xaxis.set_major_formatter(ScalarFormatter())
        axs[i].ticklabel_format(useOffset=False, style="plain", axis="x")

    # Remove any empty subplots (if there are fewer topics than subplots)
    for j in range(i + 1, len(axs)):
        fig.delaxes(axs[j])

    # Plot histogram.
    plt.tight_layout()  # Adjust layout to prevent overlap
    output_file = os.path.join(output_dir, f"{name}.png")
    plt.savefig(output_file)
    plt.close()

    print(f"   Histograms saved to {output_file}")

    run_rosbag_command(rosbag_path, output_txt_file)


if __name__ == "__main__":
    rosbag_paths = [str(s) for s in pathlib.Path(MISSION_DATA).rglob("*.bag")]
    output_dir = "topic_freq"

    for rosbag_path in rosbag_paths:
        output_dir = Path(rosbag_path).parent.joinpath(output_dir)
        output_dir.mkdir(exist_ok=True)

        name = rosbag_path.split("/")[-1].split(".")[0]
        read_rosbag_and_generate_histograms(rosbag_path, str(output_dir), name)
