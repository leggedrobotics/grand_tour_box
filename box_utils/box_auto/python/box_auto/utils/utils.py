import os
from github import Github
from pathlib import Path
import psutil
from time import sleep
from sortedcontainers import SortedDict
from collections import defaultdict
import rosbag
from typing import List, Optional

WS = os.environ.get("WS", "/home/catkin_ws")
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


def get_bag(
    pattern, directory=MISSION_DATA, auto_download=True, return_list=False, rglob=False, return_upon_no_files=False
):
    """
    Finds the matching .bag files in the mission on Kleinram and downloads it.

    If KLEINKRAM_ACTIVE is 'ACTIVE' and `auto_download` is True, it tries downloading
    using the MISSION_UUID environment variable.

    Args:
        pattern (str): File pattern (e.g. '*.bag').
        directory (str, optional): Directory to search or download to. Defaults to MISSION_DATA.
        auto_download (bool, optional): Download from Kleinkram if active. Defaults to True.

    Returns:
        str: Path to the single matched bag file.

    Raises:
        ValueError: If zero or multiple files match `pattern`.
    """

    # Kleinkram currently only supports .bag and .mcap files
    if (auto_download) and (not (pattern.endswith(".bag") or pattern.endswith(".mcap"))):
        raise ValueError(f"Pattern must end with '.bag' or '.mcap'. Got: {pattern}")

    # Download pattern matched files from Kleinkram
    if (auto_download) and (os.environ.get("KLEINKRAM_ACTIVE", False) == "ACTIVE"):
        uuid = os.environ["MISSION_UUID"]
        os.system(f"klein download --mission {uuid} --dest {directory} '{pattern}'")

    if rglob:
        # Get reference bag path
        files = [str(s) for s in Path(directory).rglob(pattern)]
    else:
        # Get reference bag path
        files = [str(s) for s in Path(directory).glob(pattern)]

    if not files:
        if return_upon_no_files:
            return None

        raise FileNotFoundError(f"No matching bags found: {pattern} in directory {directory}. \n")

    if len(files) > 1:
        if return_list:
            return files

        raise ValueError(
            f"Error: More or less matching bag files found: {pattern} in directory {directory}: \n" + str(files)
        )

    return files[0]


def find_and_extract_non_matching(directory, pattern):
    """
    Find files matching the pattern in the specified directory and extract the part of the filename
    that does not contain the matching part of the pattern.

    Args:
        directory (str): The directory to search.
        pattern (str): The pattern to match (e.g., "*_evo.tum").

    Returns:
        str: A string containing the non-matching part of the filename.
    """

    matched_files = list(Path(directory).rglob(pattern))  # Find all files matching the pattern

    if len(matched_files) == 0:
        raise ValueError(f"No files found matching the pattern '{pattern}' in directory '{directory}'.")
    elif len(matched_files) > 1:
        raise ValueError(f"Multiple files found matching the pattern '{pattern}': {[f.name for f in matched_files]}")

    filename = matched_files[0].name

    # Remove wildcard and suffix from the pattern to isolate the part to exclude
    pattern_to_exclude = pattern.replace("*", "")
    if pattern_to_exclude in filename:
        # Extract the part of the filename before the pattern to exclude
        non_matching = filename.split(pattern_to_exclude)[0]  # Take only the part before the pattern
    else:
        raise ValueError(f"Pattern '{pattern_to_exclude}' not found in filename '{filename}'")

    return non_matching


def get_file(pattern, directory=MISSION_DATA, rglob=False):
    """
    Find a single file matching the given pattern in the specified directory.

    Args:
        directory (str): Directory to search
        pattern (str): File pattern to match

    Returns:
        tuple: (file path, success flag)
    """

    if rglob:
        files = list(Path(directory).rglob(pattern))
    else:
        files = list(Path(directory).glob(pattern))

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
            if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
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

def create_github_issue(
    title: str,
    body: str,
    repo: str = "leggedrobotics/grand_tour_box",
    label: str = "auto-created",
):
    """
    Create a GitHub issue in the specified repository using PyGithub, optionally including images.

    Args:
        title (str): Title of the issue.
        body (str): Body of the issue.
        repo (str): Repository in the format "owner/repo".
        label (str): Label to differentiate auto-created issues (default: "auto-created").
    """
    token = os.getenv("GITHUB_TOKEN")
    if not token:
        raise EnvironmentError("GITHUB_TOKEN environment variable not set.")

    # Authenticate with GitHub using the personal access token
    g = Github(token)

    # Get the repository
    try:
        repository = g.get_repo(repo)
    except Exception as e:
        raise Exception(f"Could not access repository {repo}: {e}")

    # Ensure the label exists in the repository
    try:
        existing_labels = [l.name for l in repository.get_labels()]
        if label not in existing_labels:
            repository.create_label(name=label, color="00aaff", description="Auto-created issues")
    except Exception as e:
        raise Exception(f"Failed to ensure label '{label}' exists in repository {repo}: {e}")
    
    # Append a link to kleinkram onto the body
    body += f"\n\n[Link to Kleinkram](https://datasets.leggedrobotics.com/actions?sortBy=createdAt&descending=true&project_uuid={os.environ['PROJECT_UUID']}&mission_uuid={os.environ['MISSION_UUID']})"
    body += f"\nFollow the link, select the action and download the artifacts to see any images/logs not included in this issue."

    # Create the issue
    try:
        issue = repository.create_issue(title=title, body=body, labels=[label])
        print(f"Issue created: {issue.html_url}")
    except Exception as e:
        raise Exception(f"Failed to create GitHub issue: {e}")