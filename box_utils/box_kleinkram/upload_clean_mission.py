from pathlib import Path
from box_auto.utils import MISSION_DATA, get_bag, get_uuid_mapping, read_sheet_data
import rosbag
import rospy
from collections import defaultdict
from box_auto.utils import upload_simple

# Define the spreadsheet ID and sheet name
SPREADSHEET_ID = "1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg"
# Read the data and print the list
topic_data, mission_data = read_sheet_data(SPREADSHEET_ID)


# Create a dictionary with bag_name_out as the key and a list of missions as the value
data_dict_by_bag_name = defaultdict(list)
for entry in topic_data:
    data_dict_by_bag_name[entry["bag_name_out"]].append(entry)


import os

mission_name = Path(MISSION_DATA).stem

os.environ["MISSION_UUID"] = get_uuid_mapping()[mission_name]["uuid"]


if mission_name not in mission_data:
    print("Mission not found in sheet")
    exit(1)

if mission_data[mission_name]["GOOD_MISSION"] != "TRUE":
    print("Skip processing mission")
    exit(2)


out_dir_bag = Path(MISSION_DATA) / "publish_bags"
out_dir_bag.mkdir(exist_ok=True, parents=True)

exit_code = 0

error_list = {}
secret_gnss_topics = [
    "/gt_box/inertial_explorer/dgps/origin",
    "/gt_box/inertial_explorer/dgps/raw",
    "/gt_box/inertial_explorer/lc/origin",
    "/gt_box/inertial_explorer/lc/raw",
    "/gt_box/inertial_explorer/ppp/origin",
    "/gt_box/inertial_explorer/ppp/raw",
    "/gt_box/inertial_explorer/tc/origin",
    "/gt_box/inertial_explorer/tc/raw",
]

# Do a quick sorting operation
for output_bag_name, topic_configs in data_dict_by_bag_name.items():
    bag_path_out = out_dir_bag / (mission_name + "_" + output_bag_name)
    # open bag with fzf compression

    with rosbag.Bag(bag_path_out, "w") as bag_out:
        print(output_bag_name)
        for topic_config in topic_configs:
            print(
                "   process: ",
                topic_config["bag_name_orig"],
                " - ",
                topic_config["topic_name_orig"],
                " -> ",
                topic_config["topic_name_out"],
            )
            try:
                bag_path_in = get_bag("*" + topic_config["bag_name_orig"])

                with rosbag.Bag(bag_path_in, "r", compression="lz4") as bag_in:
                    desired_start_time = rospy.Time.from_sec(float(mission_data[mission_name]["mission_start_time"]))

                    if topic_config["topic_name_orig"] == "/tf_static":
                        start_time = None
                    else:
                        start_time = desired_start_time

                    for topic, msg, t in bag_in.read_messages(
                        topics=[topic_config["topic_name_orig"]],
                        start_time=start_time,
                        end_time=rospy.Time.from_sec(float(mission_data[mission_name]["mission_stop_time"])),
                    ):
                        has_header = hasattr(msg, "header")

                        if has_header:
                            t = msg.header.stamp
                        else:
                            if type(msg)._type == "tf2_msgs/TFMessage":
                                t = msg.transforms[0].header.stamp

                        if topic_config["frame_id_out"] != "":
                            msg.header.frame_id = topic_config["frame_id_out"]

                        if mission_data[mission_name]["publish_gnss"] == "FALSE" and topic in secret_gnss_topics:
                            continue

                        if topic_config["topic_name_orig"] == "/tf_static" and t < desired_start_time:
                            t = desired_start_time
                            for transform in msg.transforms:
                                transform.header.stamp = t

                        bag_out.write(topic_config["topic_name_out"], msg, t)

                # subprocess.run(["rosbag", "reindex", str(bag_path_out)])
                # print(f"Bag {bag_path_out} created and reindexed \n \n")

                upload_simple("GrandTour", "release_" + mission_name, bag_path_out)

            except Exception as e:
                if topic_config["bag_name_out"] in error_list:
                    error_list[topic_config["bag_name_out"]] += topic_config
                else:
                    error_list[topic_config["bag_name_out"]] = [topic_config]

                print(f"Error processing bag {topic_config['bag_name_orig']}")
                exit_code = 3
                print(e)


print(error_list)
exit(exit_code)
