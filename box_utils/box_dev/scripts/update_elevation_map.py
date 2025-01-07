#!/usr/bin/env python3

import rospy
from grid_map_msgs.msg import GridMap
import numpy as np
from dynamic_reconfigure.server import Server
from box_dev.cfg import FilterConfig  # You'll need to create this


class ElevationMapAdjuster:
    def __init__(self):
        rospy.init_node("elevation_map_adjuster")

        # Initialize filter parameters
        self.min_x = 0
        self.max_x = 1
        self.min_y = 0
        self.max_y = 1

        # Store last received message
        self.last_msg = None

        # Create subscriber and publisher
        self.sub = rospy.Subscriber("/gt_box/elevation_mapping/elevation_map_raw", GridMap, self.map_callback)

        self.pub = rospy.Publisher("/gt_box/elevation_mapping/elevation_map_adjusted", GridMap, queue_size=1)

        # Setup dynamic reconfigure
        self.server = Server(FilterConfig, self.reconfigure_callback)

    def reconfigure_callback(self, config, level):
        self.min_x = int(max(0, config.min_x))
        self.min_y = int(max(0, config.min_y))
        try:

            self.max_x = int(
                max(self.min_x + 1, min(int(self.last_msg.info.length_x / self.last_msg.info.resolution), config.max_x))
            )
            self.max_y = int(
                max(self.min_y + 1, min(int(self.last_msg.info.length_y / self.last_msg.info.resolution), config.max_y))
            )
        except Exception as e:
            print(e)
        # Republish last message with new filter if available
        if self.last_msg is not None:
            self.process_and_publish_msg(self.last_msg)
        else:
            self.min_x, self.max_x, self.min_y, self.max_y = 276, 349, 237, 296

        config.min_x = self.min_x
        config.min_y = self.min_y
        config.max_x = self.max_x
        config.max_y = self.max_y

        return config

    def process_and_publish_msg(self, msg):
        # Get the elevation layer data
        elevation_layer_idx = msg.layers.index("elevation")
        elevation_data = np.array(msg.data[elevation_layer_idx].data)

        # Get map metadata
        resolution = msg.info.resolution
        width = msg.info.length_x
        height = msg.info.length_y

        # Convert data to 2D array
        elevation_2d = elevation_data.reshape((int(height / resolution), int(width / resolution)))

        # Calculate grid indices for the filter rectangle

        print(self.min_x, self.max_x, self.min_y, self.max_y)
        # Set region to NaN
        elevation_2d[self.min_x : self.max_x, self.min_y : self.max_y] = np.nan
        # Update message
        msg.data[elevation_layer_idx].data = elevation_2d.flatten().tolist()

        # Publish modified map
        self.pub.publish(msg)

    def map_callback(self, msg):
        self.last_msg = msg
        self.process_and_publish_msg(msg)


if __name__ == "__main__":
    try:
        adjuster = ElevationMapAdjuster()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
