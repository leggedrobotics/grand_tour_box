import rosbag
from box_auto.utils import get_bag
import os


def bag_exists(bag_path):
    return os.path.isfile(bag_path)


def count_messages(bag_path, topic):
    count = 0
    with rosbag.Bag(bag_path, "r") as bag:
        for _, _, _ in bag.read_messages(topics=[topic]):
            count += 1
    return count


robot_bag_path = get_bag("*_jetson_ap20_robot.bag")
sync_bag_path = get_bag("*_jetson_ap20_synced.bag")
aux_bag_path = get_bag("*_jetson_ap20_aux.bag")

aux_bag_message_count = count_messages(aux_bag_path, "/gt_box/ap20/position_debug")
sync_bag_message_count = count_messages(sync_bag_path, "/gt_box/ap20/prism_position")
robot_bag_message_count = count_messages(robot_bag_path, "/gt_box/ap20/prism_position")

print(f"Sync: {sync_bag_message_count}")
print(f"Aux: {aux_bag_message_count}")
print(f"Robot: {robot_bag_message_count}")
