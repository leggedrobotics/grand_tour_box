from box_auto.utils import get_bag, BOX_AUTO_DIR, deployments, get_file
from box_auto.scripts.ap20 import process_prism_position_bag
import rosbag
import rospy
import os
import yaml
import numpy as np


def create_metadata():
    metadata = {}
    for deployment in deployments:
        for j, mission_name in enumerate(deployment["mission_names"]):
            p = os.path.join(deployment["data_folder"], mission_name)
            print(p)
            bag_path = get_bag("*_lpc_state_estimator.bag", directory=str(p))

            # Iterate over rosbag and find the first time when the robot moved in x,y L2 norm more than 0.5m from the start
            start_time = None
            stop_time = None
            distance_walked = 0.0
            previous_position = None
            print(mission_name, bag_path)

            with rosbag.Bag(bag_path, "r") as bag:
                for topic, msg, t in bag.read_messages(topics=["/state_estimator/pose_in_odom"]):
                    # Calculate L2 norm of the movement
                    if start_time is None:
                        start_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
                        start_time = t
                    else:
                        current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
                        distance = (
                            (current_position[0] - start_position[0]) ** 2
                            + (current_position[1] - start_position[1]) ** 2
                        ) ** 0.5
                        if distance > 0.5:
                            mission_start_time = t - rospy.Duration(10)
                            break

                # Compute the total distance walked
                for topic, msg, t in bag.read_messages(topics=["/state_estimator/pose_in_odom"]):
                    current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
                    if previous_position is not None:
                        step_distance = (
                            (current_position[0] - previous_position[0]) ** 2
                            + (current_position[1] - previous_position[1]) ** 2
                        ) ** 0.5
                        distance_walked += step_distance
                    previous_position = current_position

                # Find the time the robot stopped moving (difference less than 0.1m in x,y L2 norm from last position)
                for topic, msg, t in reversed(list(bag.read_messages(topics=["/state_estimator/pose_in_odom"]))):
                    current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
                    if stop_time is None:
                        stop_position = current_position
                        stop_time = t
                    else:
                        distance = (
                            (current_position[0] - stop_position[0]) ** 2
                            + (current_position[1] - stop_position[1]) ** 2
                        ) ** 0.5
                        if distance < 0.1:
                            mission_stop_time = t
                            break

            try:
                input_bag_path = get_bag("*_jetson_ap20_synced.bag", p)
                output_bag_path = input_bag_path.replace("_synced.bag", "_robot.bag")
                # Process the bag

                _, media_diff = process_prism_position_bag(
                    input_bag_path, output_bag_path, skip_check=False, remove_start=False, return_check_only=True
                )
                media_diff = media_diff.tolist()
            except Exception as e:
                media_diff = [np.inf, np.inf, np.inf]
                print(f"Error {mission_name}: {e}")

            _, gnss_dgps = get_file("*_cpt7_ie_dgps.bag", directory=str(p))
            _, gnss_ppp = get_file("*_cpt7_ie_ppp.bag", directory=str(p))
            _, gnss_tc = get_file("*_cpt7_ie_tc.bag", directory=str(p))
            _, gnss_lc = get_file("*_cpt7_ie_ls.bag", directory=str(p))

            # Fill in the bag_start_time and bag_stop_time
            metadata[mission_name] = {
                "data_folder": deployment["data_folder"],
                "mission_start_time": mission_start_time.to_sec(),
                "mission_stop_time": mission_stop_time.to_sec(),
                "mission_duration": (mission_stop_time - mission_start_time).to_sec(),
                "bag_start_time": start_time.to_sec(),
                "bag_stop_time": stop_time.to_sec(),
                "bag_duration": (stop_time - start_time).to_sec(),
                "distance_walked": distance_walked,
                "ap20_bag_available": True,
                "prism_distance": media_diff,
                "prism": "Working",
                "gnss_dgps": gnss_dgps,
                "gnss_ppp": gnss_ppp,
                "gnss_tc": gnss_tc,
                "gnss_lc": gnss_lc,
            }

    for mission_name in ["2024-10-29-09-08-34", "2024-11-04-14-55-02", "2024-11-18-16-45-27"]:
        metadata[mission_name]["ap20_bag_available"] = False
        # Schanzengraben
        # Grindelwald Glacier Canyon - Walk out
        # 2024-11-18-16-45-27 -> Totalstation clearly moved [2.06177235e-02 2.38358483e-01 2.13384628e-05]

    for mission_name in [
        "2024-10-01-11-29-55",
        "2024-10-01-11-47-44",
        "2024-10-01-12-00-49",
        "2024-12-09-09-34-43",
        "2024-12-09-09-41-46",
        "2024-12-09-11-53-11",
        "2024-12-09-11-28-28",
        "2024-11-04-14-19-11",
        "2024-10-29-09-53-44",
        "2024-10-29-09-08-34",
    ]:
        metadata[mission_name]["prism"] = "Not at start and end"

    for mission_name in [
        "2024-11-03-07-52-45",
        "2024-11-03-08-17-23",
        "2024-11-11-12-42-47",
        "2024-11-11-13-06-23",
        "2024-11-25-11-44-05",
    ]:
        metadata[mission_name]["prism"] = "Not at end"

    print(metadata[mission_name])
    # Sort the keys and dump to YAML file
    yaml_path = os.path.join(BOX_AUTO_DIR, "cfg/mission_metadata.yaml")
    with open(yaml_path, "w") as yaml_file:
        yaml.dump(metadata, yaml_file, default_flow_style=False)


if __name__ == "__main__":
    # execute_command_per_mission("python3 -m box_auto.scripts.run_mission")
    create_metadata()
