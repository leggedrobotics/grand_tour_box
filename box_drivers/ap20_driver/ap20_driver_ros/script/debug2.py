#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Imu


class TimestampAnalyzer:
    def __init__(self, target_count=100000):
        self.target_count = target_count
        self.header_timestamps = []
        self.received_timestamps = []
        self.subscriber = rospy.Subscriber("/ap20/imu", Imu, self.imu_callback)

    def imu_callback(self, msg):
        if len(self.header_timestamps) < self.target_count:
            self.header_timestamps.append(msg.header.stamp.to_sec())
            self.received_timestamps.append(rospy.Time.now().to_sec())

            if len(self.header_timestamps) % 1000 == 0:
                rospy.loginfo(f"Collected {len(self.header_timestamps)} timestamps")

        if len(self.header_timestamps) >= self.target_count:
            self.subscriber.unregister()
            self.analyze_timestamps()

    def calculate_time_distances(self, timestamps):
        return np.diff(timestamps)

    def bin_time_distances(self, time_distances, num_bins=400, bin_size=0.001):
        bins = np.arange(0, num_bins * bin_size, bin_size)
        hist, _ = np.histogram(time_distances, bins=bins)
        return hist, bins[:-1]

    def analyze_timestamps(self):
        for name, timestamps in [("Header", self.header_timestamps), ("Received", self.received_timestamps)]:
            rospy.loginfo(f"\nAnalyzing {name} Timestamps:")
            time_distances = self.calculate_time_distances(timestamps)
            hist, bin_edges = self.bin_time_distances(time_distances)

            rospy.loginfo("Time distance statistics:")
            rospy.loginfo(f"Min: {time_distances.min():.9f} seconds")
            rospy.loginfo(f"Max: {time_distances.max():.9f} seconds")
            rospy.loginfo(f"Mean: {time_distances.mean():.9f} seconds")
            rospy.loginfo(f"Median: {np.median(time_distances):.9f} seconds")

            rospy.loginfo("\nBinned time distances (1ms bins from 1ms to 400ms):")
            rospy.loginfo("Bin Start (ms) | Count")
            rospy.loginfo("---------------|------")
            for bin_start, count in zip(bin_edges, hist):
                rospy.loginfo(f"{bin_start*1000:13.1f} | {count:5d}")

        rospy.signal_shutdown("Analysis complete")


def main():
    rospy.init_node("timestamp_analyzer")
    _ = TimestampAnalyzer()
    rospy.spin()


if __name__ == "__main__":
    main()
