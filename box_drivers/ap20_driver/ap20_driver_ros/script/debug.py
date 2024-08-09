import time
import numpy as np


def read_timestamps(filename, target_count):
    timestamps = []
    while len(timestamps) < target_count:
        with open(filename, "r") as file:
            new_timestamps = [float(ts.strip()) for ts in file.readlines() if ts.strip()]
            timestamps.extend(new_timestamps)
            print(f"Read {len(new_timestamps)} timestamps, total: {len(timestamps)}")
        time.sleep(0.001)  # Small delay to avoid excessive CPU usage
    return timestamps[:target_count]  # Ensure we return exactly target_count timestamps


def calculate_time_distances(timestamps):
    return np.diff(timestamps)


def bin_time_distances(time_distances, num_bins=400, bin_size=0.001):
    bins = np.arange(0, num_bins * bin_size, bin_size)
    hist, _ = np.histogram(time_distances, bins=bins)
    return hist, bins[:-1]


def main():
    filename = "/sys/kernel/time_stamper/ts_buffer"
    target_count = 100000

    print(f"Reading {target_count} timestamps from {filename}...")
    timestamps = read_timestamps(filename, target_count)

    print("Calculating time distances...")
    time_distances = calculate_time_distances(timestamps)

    print("Binning time distances...")
    hist, bin_edges = bin_time_distances(time_distances)

    print("\nTime distance statistics:")
    print(f"Min: {time_distances.min():.9f} seconds")
    print(f"Max: {time_distances.max():.9f} seconds")
    print(f"Mean: {time_distances.mean():.9f} seconds")
    print(f"Median: {np.median(time_distances):.9f} seconds")

    print("\nBinned time distances (1ms bins from 1ms to 400ms):")
    print("Bin Start (ms) | Count")
    print("---------------|------")
    for bin_start, count in zip(bin_edges, hist):
        print(f"{bin_start*1000:13.1f} | {count:5d}")


if __name__ == "__main__":
    main()
