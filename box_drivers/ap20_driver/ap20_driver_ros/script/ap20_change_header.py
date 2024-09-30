import rosbag
from std_msgs.msg import Time
import os
import rospy
def modify_bag(input_bag_path):
    # Create output bag path
    base, ext = os.path.splitext(input_bag_path)
    output_bag_path = f"{base}_header{ext}"

    with rosbag.Bag(input_bag_path, 'r') as inbag, rosbag.Bag(output_bag_path, 'w') as outbag:
        for topic, msg, t in inbag.read_messages():
            if topic == '/gt_box/ap20/timestamp_debug':
                # Create a new message with the timestamp in the header
                new_msg = msg
                new_msg.header.stamp = new_msg.timestamp.data
                outbag.write('/gt_box/ap20/timestamp_debug_header', new_msg, t)
            else:
                # Write all other messages as they are
                outbag.write(topic, msg, t)

    print(f"Modified bag saved as: {output_bag_path}")

if __name__ == "__main__":
    bag_path = '/Data/Projects/GrandTour/ap20_matching_data/2024-09-28-19-33-31_jetson_ap20_aux_0.bag'
    modify_bag(bag_path)