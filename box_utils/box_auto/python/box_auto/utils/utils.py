import os
from github import Github
from pathlib import Path
import psutil
from time import sleep
from sortedcontainers import SortedDict
from collections import defaultdict
import rosbag

WS = os.environ.get("WS", "/home/catkin_ws")
PRE = f"source /opt/ros/noetic/setup.bash; source {WS}/devel/setup.bash; "
MISSION_DATA = os.environ.get("MISSION_DATA", "/tmp_disk")
BOX_AUTO_SCRIPTS_DIR = str(Path(__file__).parent.parent / "scripts")
BOX_AUTO_DIR = str(Path(__file__).parent.parent.parent.parent)

if os.environ.get("KLEINKRAM_ACTIVE", False) == "ACTIVE":
    ARTIFACT_FOLDER = os.environ.get("ARTIFACT_FOLDER", "/out")
else:
    ARTIFACT_FOLDER = os.environ.get("ARTIFACT_FOLDER", MISSION_DATA)

UUID_MAPPING = {
    "2024-11-18-17-13-09": {
        "uuid": "00e59f99-c98f-4cf4-b1b4-6a90a1218a73",
        "uuid_pub": "5e0afa5b-605f-49e0-842c-defea1686360",
    },
    "2024-11-14-12-01-26": {
        "uuid": "0df5636d-6612-4296-8b46-d713ffe0e358",
        "uuid_pub": "bac8fa18-4ed0-41d7-904f-c5c7ccdfe0eb",
    },
    "2024-11-03-14-36-54": {
        "uuid": "120ed19f-7928-4701-bfbd-bed46c4a6403",
        "uuid_pub": "0a47546c-6ca5-4535-95fe-12114a13d562",
    },
    "2024-11-18-17-31-36": {
        "uuid": "12139d35-148d-4b27-8232-5116a4ac9a10",
        "uuid_pub": "822b83b8-240f-4703-9395-8315f936198d",
    },
    "2024-10-01-11-47-44": {
        "uuid": "1431d8d4-ec3f-485f-8518-76a9f75c65ea",
        "uuid_pub": "7672c5e8-2123-4d9e-89cc-3c18d3ae5031",
    },
    "2024-11-18-12-05-01": {
        "uuid": "15dbe938-35d2-4b80-968c-5e809160cd43",
        "uuid_pub": "04e883df-9bb0-45dc-8237-1f442dc214d3",
    },
    "2024-11-11-16-14-23": {
        "uuid": "1942c776-82dd-4dcf-ace2-7ff5738769e2",
        "uuid_pub": "c6ea1b53-90d9-449c-99ab-9489de6eee39",
    },
    "2024-12-03-13-26-40": {
        "uuid": "1c6c9d86-606d-476c-9226-02da290eb35d",
        "uuid_pub": "33507854-8f20-4f69-a809-9f781d8a505d",
    },
    "2024-11-04-16-05-00": {
        "uuid": "1d436835-4cc3-4351-a35d-270b57880d75",
        "uuid_pub": "6312e468-fe07-43df-8619-81c3629a7f95",
    },
    "2024-11-02-20-26-48": {
        "uuid": "23eb33b1-3cf2-4791-8fd6-cd1d4fc28686",
        "uuid_pub": "779bfc40-1fb9-4dc4-8c81-7bd23ecd2539",
    },
    "2024-11-03-07-57-34": {
        "uuid": "25bfe0c8-96ff-43d1-8b51-e044a21b53c3",
        "uuid_pub": "b5e4452a-8072-4c66-8f61-0d0e2eb4f168",
    },
    "2024-11-04-10-57-34": {
        "uuid": "3154972c-624e-4a94-95bf-83c76ca5355b",
        "uuid_pub": "a18bbc85-045e-4226-8c4c-0d2ca4a60255",
    },
    "2024-11-03-08-17-23": {
        "uuid": "34f19ac3-7f35-4cf4-98f6-b61a912f7ca6",
        "uuid_pub": "2e74b1d6-24ff-45a7-8109-b79c12057489",
    },
    "2024-11-02-21-12-51": {
        "uuid": "3e451087-704e-41d6-865e-5f7c96276690",
        "uuid_pub": "21c99bdd-d285-4b90-a2a5-34ea08e02197",
    },
    "2024-11-18-16-59-23": {
        "uuid": "411fff04-fafb-4115-a123-4fa3c06c9e48",
        "uuid_pub": "8c401c01-0917-4661-9466-09bbe08b42d0",
    },
    "2024-11-14-11-17-02": {
        "uuid": "432d6c4f-8dc1-4c23-95e4-bddfe0e32ad4",
        "uuid_pub": "52919611-6abe-4514-b99b-975ccf01692d",
    },
    "2024-10-29-16-11-44": {
        "uuid": "43457f5f-d559-4d82-9d29-209dae089671",
        "uuid_pub": "25355d63-69d2-41c2-899b-1b246e1511b9",
    },
    "2024-11-11-14-29-44": {
        "uuid": "45041a33-1410-4eea-a506-75aff00eb30d",
        "uuid_pub": "345a9d53-af6a-4c60-bcaf-a536bc1834c2",
    },
    "2024-11-15-13-56-45": {
        "uuid": "4504bec7-0e28-4cb5-819c-8bc38f711d12",
        "uuid_pub": "312946be-02f2-4019-8bda-0af667947558",
    },
    "2024-11-18-16-45-27": {
        "uuid": "4d896ffa-2075-4402-b5cc-068a96e684ab",
        "uuid_pub": "3b0910da-aeec-4dff-b654-b2a1aff27a08",
    },
    "2024-11-15-11-37-15": {
        "uuid": "4de26aa4-5b54-412b-bb99-3c7ca1b05bf5",
        "uuid_pub": "1bb49a5f-db56-4f65-b3e2-811e094371f8",
    },
    "2024-11-02-17-10-25": {
        "uuid": "505853a7-82ce-4408-a603-473b579da622",
        "uuid_pub": "8cf69b39-e669-4cca-91b5-d07a32a7b3df",
    },
    "2024-12-09-09-41-46": {
        "uuid": "5536d4b8-27e0-495d-becc-03cd80e46de0",
        "uuid_pub": "01e8ce70-2068-4ca8-a11f-0050c0eef96b",
    },
    "2024-11-18-13-48-19": {
        "uuid": "58bdee5a-0d4c-4ad2-afca-26a81bb5226b",
        "uuid_pub": "1eb04ef2-c0d2-431c-b09a-618bd4b19a69",
    },
    "2024-11-02-17-43-10": {
        "uuid": "5c258464-4797-4080-ad8b-780fc02410d7",
        "uuid_pub": "8e2f36cb-6895-4a4b-b667-df61b037c181",
    },
    "2024-11-14-16-04-09": {
        "uuid": "6a6c87ce-9b85-4c56-b4d6-b37453a94ed9",
        "uuid_pub": "c0eafdd6-72fe-4c3c-8310-78a23e367040",
    },
    "2024-11-04-16-52-38": {
        "uuid": "6da55a8b-a3a5-42a8-a496-53dee287e431",
        "uuid_pub": "cdf624f5-baa7-4a84-b1a5-e8b839675381",
    },
    "2024-11-11-12-07-40": {
        "uuid": "79a64a02-f81e-4150-8de7-d5904ae2bd81",
        "uuid_pub": "78660b51-559d-4acb-a7eb-78394e78407b",
    },
    "2024-10-01-11-29-55": {
        "uuid": "79f6c6b8-9e48-4c52-b31d-446d0f5de013",
        "uuid_pub": "5364ceeb-0955-4135-8f6a-7515798b373d",
    },
    "2024-12-09-11-53-11": {
        "uuid": "7a5c5fd5-683d-44a4-a27e-af08ea0229ab",
        "uuid_pub": "528c7a33-25d6-439f-8f1c-40caa38d3ed2",
    },
    "2024-11-04-12-55-59": {
        "uuid": "7b1784ab-8045-4da0-bb69-b1a6c644361d",
        "uuid_pub": "3667c02e-d4d3-4209-8906-e1e21744dc6c",
    },
    "2024-10-29-09-53-44": {
        "uuid": "7b91d33c-2dce-4194-9df3-d6b7ef8950cc",
        "uuid_pub": "bf90035a-0f2c-4948-9ca0-7ebc027f26dd",
    },
    "2024-11-21-16-36-19": {
        "uuid": "7e127b00-dd50-413d-9728-29856c6943b6",
        "uuid_pub": "bc4cac53-8b10-48eb-b638-15705ca98706",
    },
    "2024-12-09-09-34-43": {
        "uuid": "83588d12-cb89-4034-b4f7-6e13ca74cbce",
        "uuid_pub": "3d1541f6-1ddb-42d7-bbfa-50164a784cc3",
    },
    "2024-11-25-16-02-15": {
        "uuid": "8693e856-9347-4295-a139-41ab518a0f6f",
        "uuid_pub": "c9ab8186-f4c4-43e5-ae88-6c89d594f6ca",
    },
    "2024-11-03-13-51-43": {
        "uuid": "86d7cf0c-62c5-4ab2-ae90-8288072ba96d",
        "uuid_pub": "a0a17b4f-21eb-4edd-a931-c03a72272e06",
    },
    "2024-10-01-12-00-49": {
        "uuid": "8a088084-fe3e-43fe-bf2c-498c98a054d8",
        "uuid_pub": "7419d748-d1a4-41e6-8446-385d8e1930e4",
    },
    "2024-11-11-13-06-23": {
        "uuid": "8d5d7e8f-109d-468e-8f52-274d24020b3a",
        "uuid_pub": "739a5963-7180-431a-8644-ff9e2f51ab07",
    },
    "2024-11-18-15-46-05": {
        "uuid": "8ef688b4-cde8-4f7d-b366-fce10d11a455",
        "uuid_pub": "8643562f-ba43-4420-8714-8049790d0501",
    },
    "2024-11-18-13-22-14": {
        "uuid": "93ac4636-de7a-43cf-8057-7f25755f7be9",
        "uuid_pub": "3eaa97b8-bcb4-4a26-a20a-c21d8ef87a3b",
    },
    "2024-11-15-14-14-12": {
        "uuid": "94b2d7c6-5d20-4e9a-9c4a-2e58064faf5c",
        "uuid_pub": "2af31f82-235f-4b72-8368-ccc8e9af17bb",
    },
    "2024-11-04-14-19-11": {
        "uuid": "9b10e9db-ab23-4cb2-8487-fa5403d45913",
        "uuid_pub": "badb38bd-5674-4ad8-872d-734bf1fbbd6e",
    },
    "2024-11-02-19-47-44": {
        "uuid": "9c1e0fde-53a3-49f5-a943-0bdcb9c1de4a",
        "uuid_pub": "4c223749-1033-4637-a187-feedc6a07d04",
    },
    "2024-11-25-16-36-19": {
        "uuid": "a44792cd-b474-4e07-a03d-e19542e78b13",
        "uuid_pub": "5800e00d-7535-45a6-aa67-e773d4f2d570",
    },
    "2024-11-15-11-18-14": {
        "uuid": "a5499f97-c96e-49fd-a120-5e769f5f00a9",
        "uuid_pub": "1f921d9f-241a-42ff-a56e-a5c4fe06e444",
    },
    "2024-11-15-14-43-52": {
        "uuid": "a634dee5-60e5-46e3-a83f-3b062cc42895",
        "uuid_pub": "acb34749-ecb7-4c83-8d4b-407137fb1436",
    },
    "2024-11-03-08-42-30": {
        "uuid": "a83081a6-4a07-418a-8cee-943cd024325d",
        "uuid_pub": "6cec4a78-e7bb-4d9b-a7f0-a09aec24a4d0",
    },
    "2024-10-29-09-08-34": {
        "uuid": "aac088c5-ea62-404e-a21b-59775e8caa9d",
        "uuid_pub": "3abac3ec-2720-4c15-82ca-d90f315cd161",
    },
    "2024-11-15-12-06-03": {
        "uuid": "b08f0bcb-571a-4cfb-9a04-58f0c6928bdd",
        "uuid_pub": "2fd7597c-96d5-4c8d-94cf-515bfd6d6796",
    },
    "2024-11-04-14-55-02": {
        "uuid": "b243fc8b-ca51-4950-9659-1368d5385aba",
        "uuid_pub": "74f3c6cb-5a3e-4652-8eab-6afc85763ea6",
    },
    "2024-12-09-11-28-28": {
        "uuid": "b3332086-58ab-43a4-a1e5-a0d63836ac0d",
        "uuid_pub": "efa4cc8b-f37b-4f8e-87ef-f625fc1a9963",
    },
    "2024-11-03-11-03-50": {
        "uuid": "b69be373-a126-4a0e-85e9-754fa41ec1da",
        "uuid_pub": "46af69c2-95cd-4012-a819-388e8bfd7e62",
    },
    "2024-11-03-07-52-45": {
        "uuid": "bb6795bd-e38d-4085-bd7b-22248a430f29",
        "uuid_pub": "8eebbe31-9051-4b70-b8c3-ecf9166d1a78",
    },
    "2024-11-04-11-56-53": {
        "uuid": "be0c6647-3f46-447c-b96b-7816ee33e40d",
        "uuid_pub": "a172a021-16eb-408e-b4d2-4accf28600ec",
    },
    "2024-11-01-17-46-15": {
        "uuid": "c00eaf5b-42bc-4d6c-a0b2-086e78d16f0c",
        "uuid_pub": "0840eb04-616a-486b-980a-21c47015fefe",
    },
    "2024-11-18-11-42-04": {
        "uuid": "c10798d4-2690-4076-9f70-130b211adee8",
        "uuid_pub": "7c8bd26c-76ac-46a5-82f4-3f84ffa34ad1",
    },
    "2024-11-03-13-59-54": {
        "uuid": "c171bee4-28f4-43c3-827a-ffd7c18583bc",
        "uuid_pub": "fe090e00-6a41-4963-a41b-0787840cfb60",
    },
    "2024-11-14-14-36-02": {
        "uuid": "c513f500-82ad-486c-aebd-6b1ae728ec94",
        "uuid_pub": "f3889925-183f-406d-a2ef-12870e7af5e8",
    },
    "2024-11-04-13-07-13": {
        "uuid": "ca6db1b8-f7b4-4498-a0b0-679eb16cef62",
        "uuid_pub": "c2a18325-66c1-43bc-8e36-b10bf40ba6c1",
    },
    "2024-11-02-17-18-32": {
        "uuid": "cdf8617f-d643-4bb4-9c8f-976c04dafafd",
        "uuid_pub": "b62d3fc4-76fc-41d9-a275-5dff9a0de1a9",
    },
    "2024-11-15-15-07-36": {
        "uuid": "cf4bdddc-7ed4-444a-8d7a-de59364693eb",
        "uuid_pub": "2a9fb5c3-dbc8-4763-b8f4-5bf1b01dcdea",
    },
    "2024-11-15-10-16-35": {
        "uuid": "cfa237e4-386d-4539-9ba5-667b1f176a21",
        "uuid_pub": "1e4949c0-e7e5-4e0f-8a95-7e737a41a619",
    },
    "2024-11-15-16-41-14": {
        "uuid": "d66057fc-8e40-4939-a6ce-c4833d2b2964",
        "uuid_pub": "2a1d0bec-a49d-4c2f-8154-32c2c8dc7b75",
    },
    "2024-11-25-14-57-08": {
        "uuid": "df0abba7-1e61-427f-8111-d1f77d52dddc",
        "uuid_pub": "0ab99e51-ce61-4db3-97e4-f909c4d8e302",
    },
    "2024-11-14-15-22-43": {
        "uuid": "e311107b-1c8e-44dd-abcb-ec60a59ce330",
        "uuid_pub": "b372e5c0-eba7-4380-9c9c-39154b7d0db3",
    },
    "2024-11-11-12-42-47": {
        "uuid": "e6580214-d883-4a4b-9742-37f44d20e374",
        "uuid_pub": "8f4214bc-1860-4f5c-81ee-0aa837266204",
    },
    "2024-12-03-13-15-38": {
        "uuid": "e6c30062-997a-407d-8a68-232d1f3dd505",
        "uuid_pub": "7741de7e-3c54-4f8b-970d-ccc161f4e2a7",
    },
    "2024-11-14-13-45-37": {
        "uuid": "e9a2a009-b893-4743-9d37-eed954469982",
        "uuid_pub": "278f44eb-2049-4919-bee0-c91fa850e3c1",
    },
    "2024-11-02-20-03-21": {
        "uuid": "f4962481-018e-49c2-bd2f-1a570c5c4d11",
        "uuid_pub": "e83e6871-a3ae-406f-a1fa-0e63fc9bee75",
    },
}


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


def upload_bag(bags, upload_to_pub=True):
    if os.environ.get("KLEINKRAM_ACTIVE", False) == "ACTIVE":
        uuid = os.environ["MISSION_UUID"]

        if upload_to_pub:
            uuid_map = {v["uuid"]: v["uuid_pub"] for k, v in UUID_MAPPING.items()}
            uuid = uuid_map[uuid]

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
    repo: str = "leggedrobotics/grand_tour_actions",
    label: str = "auto-created",
    dry_run: bool = False,
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
        existing_labels = [la.name for la in repository.get_labels()]
        if label not in existing_labels:
            repository.create_label(name=label, color="00aaff", description="Auto-created issues")
    except Exception as e:
        raise Exception(f"Failed to ensure label '{label}' exists in repository {repo}: {e}")

    # Append links to kleinkram onto the body
    if "ACTION_UUID" in os.environ and "MISSION_UUID" in os.environ and "PROJECT_UUID" in os.environ:
        action_uuid = os.environ.get("ACTION_UUID")
        body += f"\n\n[Link to Kleinkram Action (Logs and Artifacts)](https://datasets.leggedrobotics.com/action/{action_uuid})"
        project_uuid = os.environ.get("PROJECT_UUID")
        mission_uuid = os.environ.get("MISSION_UUID")
        body += f"\n\n[Link to Kleinkram Mission Files](https://datasets.leggedrobotics.com/project/{project_uuid}/mission/{mission_uuid}/files?sortBy=filename)"

    if dry_run:
        print(f"Would Create GitHub issue in repository {repo} with label '{label}'")
        print(f"Title: {title}")
        print(f"Body: {body}")

    else:
        # Create the issue
        try:
            issue = repository.create_issue(title=title, body=body, labels=[label])
            print(f"Issue created: {issue.html_url}")
        except Exception as e:
            raise Exception(f"Failed to create GitHub issue: {e}")
