from pathlib import Path
from box_auto.utils import get_bag, get_uuid_mapping, read_sheet_data
import rosbag
import rospy
from collections import defaultdict
from box_auto.utils import upload_simple
import subprocess
import kleinkram
import os
import shutil
import fnmatch

uuid_mappings = get_uuid_mapping()
# Define the spreadsheet ID and sheet name
SPREADSHEET_ID = "1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg"
# Read the data and print the list
topic_data, mission_data = read_sheet_data(SPREADSHEET_ID)
LIST_OF_OUTPUT_BAGS = ["zed2i_depth.bag"]
# "zed2i_prop.bag", "zed2i_vio.bag" supports ["*"] for full mission
EXISTS_SKIP = False

for name, data in uuid_mappings.items():
    try:

        if name != "2024-12-09-11-53-11" and name != "2024-12-09-11-28-28":
            continue

        data_dict_by_bag_name = defaultdict(list)
        for entry in topic_data:
            data_dict_by_bag_name[entry["bag_name_out"]].append(entry)

        if mission_data[name]["GOOD_MISSION"] != "TRUE":
            print("Skip processing mission - no good mission")
            continue

        res = kleinkram.list_files(mission_ids=[data["uuid_release"]])

        exit_code = 0

        error_list = {}
        secret_gnss_bags_out = ["cpt7_ie_tc.bag", "cpt7_ie_rt.bag"]
        tmp_folder = Path("/data") / (name + "_release")
        tmp_folder.mkdir(parents=True, exist_ok=True)

        out_dir_bag = tmp_folder / "publish_bags"
        out_dir_bag.mkdir(exist_ok=True, parents=True)

        ls = []
        data_dict_by_bag_name_filtered = {}
        for k, v in data_dict_by_bag_name.items():
            if not any(fnmatch.fnmatch(k, pattern) for pattern in LIST_OF_OUTPUT_BAGS):
                print(f"Skipping {k} as it does not match any pattern in list_of_output_bags")
                continue
            else:
                print(f"Processing {k} as it matches a pattern in list_of_output_bags")

            data_dict_by_bag_name_filtered[k] = v
            for topic_config in v:
                ls.append("*" + topic_config["bag_name_orig"])

        ls = list(set(ls))

        # Do a quick sorting operation
        bags_to_upload = []
        downloaded = False
        for output_bag_name, topic_configs in data_dict_by_bag_name_filtered.items():
            if EXISTS_SKIP:
                res = kleinkram.list_files(
                    mission_ids=[data["uuid_release"]],
                    file_names=["*" + output_bag_name],
                )
                if len(res) != 0:
                    print(f"Skipping {output_bag_name} as it already exists")
                    continue

            if not downloaded:
                kleinkram.download(
                    mission_ids=[data["uuid_pub"]],
                    file_names=ls,
                    dest=tmp_folder,
                    verbose=True,
                    overwrite=True,
                )
                downloaded = True

            bag_path_out = out_dir_bag / (name + "_" + output_bag_name)
            with rosbag.Bag(bag_path_out, "w", compression="lz4") as bag_out:
                if mission_data[name]["publish_gnss"] == "FALSE" and output_bag_name in secret_gnss_bags_out:
                    print(f"Skipping {output_bag_name} as publish_gnss is FALSE")
                    continue

                print("Writing to bag: ", output_bag_name)
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
                        bag_path_in = get_bag("*" + topic_config["bag_name_orig"], directory=tmp_folder)
                        with rosbag.Bag(bag_path_in, "r") as bag_in:
                            desired_start_time = rospy.Time.from_sec(float(mission_data[name]["mission_start_time"]))
                            desired_stop_time = rospy.Time.from_sec(float(mission_data[name]["mission_stop_time"]))

                            if topic_config["topic_name_orig"] == "/tf_static":
                                start_time = None
                            else:
                                start_time = desired_start_time

                            k = 0
                            for topic, msg, t in bag_in.read_messages(
                                topics=[topic_config["topic_name_orig"]],
                                start_time=start_time,
                                end_time=rospy.Time.from_sec(float(mission_data[name]["mission_stop_time"])),
                            ):
                                has_header = hasattr(msg, "header")

                                if has_header:
                                    t = msg.header.stamp
                                else:
                                    if type(msg)._type == "tf2_msgs/TFMessage":
                                        t = msg.transforms[0].header.stamp
                                if t > desired_stop_time:
                                    if k > 50:
                                        break
                                    k += 1
                                    continue

                                if topic_config["frame_id_out"] != "":
                                    msg.header.frame_id = topic_config["frame_id_out"]

                                if topic_config["topic_name_orig"] == "/tf_static" and t < desired_start_time:
                                    t = desired_start_time
                                    for transform in msg.transforms:
                                        transform.header.stamp = t

                                if topic_config["topic_name_out"] == "/boxi/zed2i_vio/odom":
                                    # Fix for #issues/1124
                                    msg.child_frame_id = "zed2i_base"

                                bag_out.write(topic_config["topic_name_out"], msg, t)

                    except Exception as e:
                        if topic_config["bag_name_out"] in error_list:
                            error_list[topic_config["bag_name_out"]] += topic_config
                        else:
                            error_list[topic_config["bag_name_out"]] = [topic_config]

                        print(f"Error processing bag {topic_config['bag_name_orig']}")
                        exit_code = 3
                        print(e)

            subprocess.run(["rosbag", "reindex", str(bag_path_out)])
            tmp_path = str(bag_path_out).replace(".bag", ".orig.bag")
            if os.path.exists(tmp_path):
                os.remove(tmp_path)
            bags_to_upload.append(str(bag_path_out))

        for bag_path_out in bags_to_upload:
            upload_simple("GrandTourDataset", "release_" + name, str(bag_path_out))

        print(error_list)
    except Exception as e:
        print("Got exception mission ", name, e)
    finally:
        try:
            shutil.rmtree(tmp_folder)
        except:
            pass
