import os
from pathlib import Path
import psutil
from time import sleep
from sortedcontainers import SortedDict
from collections import defaultdict
import rosbag

WS = "/home/catkin_ws"
PRE = f"source /opt/ros/noetic/setup.bash; source {WS}/devel/setup.bash; "
MISSION_DATA = os.environ.get("MISSION_DATA", "/tmp_disk")
BOX_AUTO_SCRIPTS_DIR = str(Path(__file__).parent.parent / "scripts")
BOX_AUTO_DIR = str(Path(__file__).parent.parent.parent.parent)

if os.environ.get("KLEINKRAM_ACTIVE", False) == "ACTIVE":
    ARTIFACT_FOLDER = os.environ.get("ARTIFACT_FOLDER", "/out")
else:
    ARTIFACT_FOLDER = os.environ.get("ARTIFACT_FOLDER", MISSION_DATA)


def run_ros_command(cmd, background=False):
    if background:
        cmd += " &"
    return os.system("bash -c '" + PRE + cmd + "' ")


def start_roscore():
    os.environ["ROS_MASTER_URI"] = "http://localhost:11311"
    os.system("bash -c '" + PRE + "roscore&' ")
    sleep(0.1)
    os.system("bash -c '" + PRE + "rosparam set use_sim_time true' ")


def kill_roscore():
    # Find all processes with name 'rosmaster'
    for proc in psutil.process_iter(["pid", "name"]):
        if "rosmaster" in proc.info["name"]:
            try:
                # Terminate the process
                print(f"Terminating rosmaster with PID {proc.info['pid']}")
                proc.terminate()
                proc.wait(timeout=60)  # Wait up to 60 seconds for the process to terminate
                print("rosmaster process terminated successfully.")
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.TimeoutExpired) as e:
                print(f"Failed to terminate rosmaster process: {e}")


def get_bag(pattern, directory=MISSION_DATA):
    # Get reference bag path
    if os.environ.get("KLEINKRAM_ACTIVE", False) == "ACTIVE":
        uuid = os.environ["MISSION_UUID"]
        os.system(f"klein download --mission {uuid} --dest {directory} '{pattern}'")

    files = [str(s) for s in Path(directory).rglob(pattern)]
    if len(files) != 1:
        raise ValueError(
            f"Error: More or less matching bag files found: {pattern} in directory {directory}: \n" + str(files)
        )
    return files[0]


def get_file(pattern, directory=MISSION_DATA):
    """
    Find a single file matching the given pattern in the specified directory.

    Args:
        directory (str): Directory to search
        pattern (str): File pattern to match

    Returns:
        tuple: (file path, success flag)
    """

    files = list(Path(directory).rglob(pattern))
    if len(files) != 1:
        print(f"Error: Found {len(files)} matching files for pattern {pattern} in directory {directory}")
        return None, False

    return str(files[0]), True


def upload_bag(bags):
    if os.environ.get("KLEINKRAM_ACTIVE", False) == "ACTIVE":
        uuid = os.environ["MISSION_UUID"]
        if type(bags) is list:
            bags = " ".join(bags)
        os.system(f"klein upload --mission {uuid} {bags}")
        print(f"Kleinkram Aciive - Bags uploaded: {bags}")
    else:
        print(f"Kleinkram Inactive - Bags not uploaded: {bags}")

def check_duplicate_timestamps(bag_file, topic_name):
    try:
        print(f"Reading bag file: {bag_file}")
        bag = rosbag.Bag(bag_file)
        timestamps = defaultdict(int)
        total_msgs = 0

        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                timestamp = msg.header.stamp.to_nsec()
                timestamps[timestamp] += 1
                total_msgs += 1

        bag.close()

        duplicates = {ts: count for ts, count in timestamps.items() if count > 1}
        if duplicates:
            print(f"\nFound {len(duplicates)} duplicated timestamps:")
            for ts, count in duplicates.items():
                print(f"Timestamp: {ts}, Duplicates: {count}")
            total_duplicates = sum(count - 1 for count in duplicates.values())
            print(f"\nTotal duplicate instances across all messages: {total_duplicates}")
        else:
            print("\nNo duplicated timestamps found.")

        print(f"\nProcessed {total_msgs} messages.")
    except Exception as e:
        print(f"Error: {e}")

class RosbagMessageGenerator:
    def __init__(self, bag_paths):
        """
        Initializes the generator with a list of ROS bag files.
        Only opens the bags without loading messages.
        """
        self.bags = {p: rosbag.Bag(p, "r") for p in bag_paths}
        self.bags_iterators = {p: self.bags[p].read_messages() for p in bag_paths}
        self.messages = SortedDict()
        for p in bag_paths:
            topic, msg, t = next(self.bags_iterators[p])
            k = t.to_sec()
            while k in self.messages:
                k = k + 0.0000002

            self.messages[t.to_sec()] = (topic, msg, t, p)

    def __iter__(self):
        return self

    def __next__(self):
        """Returns the message with earliest timestamp across all bags"""
        # Filter out None values and find earliest message
        if len(self.messages) == 0:
            raise StopIteration
        k, v = self.messages.popitem(0)
        topic, msg, t, bag_path = v
        try:
            topic_new, msg_new, t_new = next(self.bags_iterators[bag_path])

            k = t_new.to_sec()
            while k in self.messages:
                k = k + 0.0000002

            self.messages[k] = (topic_new, msg_new, t_new, bag_path)

        except StopIteration:
            print(f"Finished iterating over bag {bag_path}")
        return topic, msg, t

    def __del__(self):
        for p in self.bags:
            self.bags[p].close()
