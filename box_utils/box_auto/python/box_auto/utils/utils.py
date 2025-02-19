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
        "uuid_pub": "a1201085-63a6-4596-9b10-426c62ed8eaf",
    },
    "2024-11-14-12-01-26": {
        "uuid": "0df5636d-6612-4296-8b46-d713ffe0e358",
        "uuid_pub": "71008528-489c-4424-96a2-3c6a479289b4",
    },
    "2024-11-03-14-36-54": {
        "uuid": "120ed19f-7928-4701-bfbd-bed46c4a6403",
        "uuid_pub": "93a3ae26-aa4c-4aeb-a1d8-8b91f8dff9a4",
    },
    "2024-11-18-17-31-36": {
        "uuid": "12139d35-148d-4b27-8232-5116a4ac9a10",
        "uuid_pub": "e1f4e96e-be6a-40b2-9966-10bb51239fbf",
    },
    "2024-10-01-11-47-44": {
        "uuid": "1431d8d4-ec3f-485f-8518-76a9f75c65ea",
        "uuid_pub": "2365f88f-615e-476d-9749-d79ad57ea882",
    },
    "2024-11-18-12-05-01": {
        "uuid": "15dbe938-35d2-4b80-968c-5e809160cd43",
        "uuid_pub": "8a07cf3a-9dad-4015-b34a-99e6d65d9f12",
    },
    "2024-11-11-16-14-23": {
        "uuid": "1942c776-82dd-4dcf-ace2-7ff5738769e2",
        "uuid_pub": "17694a77-d03a-483e-88ac-7c8e9e4275db",
    },
    "2024-12-03-13-26-40": {
        "uuid": "1c6c9d86-606d-476c-9226-02da290eb35d",
        "uuid_pub": "078aec49-21d4-4b60-ba72-5c5cc61feec6",
    },
    "2024-11-04-16-05-00": {
        "uuid": "1d436835-4cc3-4351-a35d-270b57880d75",
        "uuid_pub": "2d931fed-5746-4227-934c-011c7f01d53d",
    },
    "2024-11-02-20-26-48": {
        "uuid": "23eb33b1-3cf2-4791-8fd6-cd1d4fc28686",
        "uuid_pub": "dac71e87-35f0-4ee6-a127-426032e556e6",
    },
    "2024-11-03-07-57-34": {
        "uuid": "25bfe0c8-96ff-43d1-8b51-e044a21b53c3",
        "uuid_pub": "b18aa7c7-0025-47f3-bb8b-f1c342077649",
    },
    "2024-11-04-10-57-34": {
        "uuid": "3154972c-624e-4a94-95bf-83c76ca5355b",
        "uuid_pub": "179fe6dd-6c60-495f-b88d-677bcc8b4e8a",
    },
    "2024-11-03-08-17-23": {
        "uuid": "34f19ac3-7f35-4cf4-98f6-b61a912f7ca6",
        "uuid_pub": "f7144fd5-298e-4286-b37c-c0a1d048b079",
    },
    "2024-11-02-21-12-51": {
        "uuid": "3e451087-704e-41d6-865e-5f7c96276690",
        "uuid_pub": "bdb8ff8b-79cb-4940-a6d0-bbed9d108dd0",
    },
    "2024-11-18-16-59-23": {
        "uuid": "411fff04-fafb-4115-a123-4fa3c06c9e48",
        "uuid_pub": "c39fdec3-afcc-45c9-8286-7f9af5a06c77",
    },
    "2024-11-14-11-17-02": {
        "uuid": "432d6c4f-8dc1-4c23-95e4-bddfe0e32ad4",
        "uuid_pub": "c9bb2c72-779d-429f-9af8-2888acd237b8",
    },
    "2024-10-29-16-11-44": {
        "uuid": "43457f5f-d559-4d82-9d29-209dae089671",
        "uuid_pub": "d64942ec-eb50-4e05-82a9-1b66d15f687b",
    },
    "2024-11-11-14-29-44": {
        "uuid": "45041a33-1410-4eea-a506-75aff00eb30d",
        "uuid_pub": "dc0f84f8-6a4f-4227-b868-dc5e5891d030",
    },
    "2024-11-15-13-56-45": {
        "uuid": "4504bec7-0e28-4cb5-819c-8bc38f711d12",
        "uuid_pub": "da9a6f30-1bd8-40ef-8cda-2aac520ddae5",
    },
    "2024-11-18-16-45-27": {
        "uuid": "4d896ffa-2075-4402-b5cc-068a96e684ab",
        "uuid_pub": "9281fab0-86ee-48e4-a7e2-01fd5c613db5",
    },
    "2024-11-15-11-37-15": {
        "uuid": "4de26aa4-5b54-412b-bb99-3c7ca1b05bf5",
        "uuid_pub": "5c2fb9f1-ec5e-4c74-b021-bad3d14ddb03",
    },
    "2024-11-02-17-10-25": {
        "uuid": "505853a7-82ce-4408-a603-473b579da622",
        "uuid_pub": "1838da8b-43cb-44ec-b418-f0ce79a72dee",
    },
    "2024-12-09-09-41-46": {
        "uuid": "5536d4b8-27e0-495d-becc-03cd80e46de0",
        "uuid_pub": "a61207ea-d815-4c9b-92ca-5e5e4b4d9b76",
    },
    "2024-11-18-13-48-19": {
        "uuid": "58bdee5a-0d4c-4ad2-afca-26a81bb5226b",
        "uuid_pub": "8de06e1b-6094-4814-9465-641c9530a230",
    },
    "2024-11-02-17-43-10": {
        "uuid": "5c258464-4797-4080-ad8b-780fc02410d7",
        "uuid_pub": "70b9485d-5271-42d4-b4ac-01674851fff5",
    },
    "2024-11-14-16-04-09": {
        "uuid": "6a6c87ce-9b85-4c56-b4d6-b37453a94ed9",
        "uuid_pub": "91784e7d-16c5-42e5-87ac-012073e9f1a6",
    },
    "2024-11-04-16-52-38": {
        "uuid": "6da55a8b-a3a5-42a8-a496-53dee287e431",
        "uuid_pub": "13b51897-e1d4-43e1-901e-a7b427db58a1",
    },
    "2024-11-11-12-07-40": {
        "uuid": "79a64a02-f81e-4150-8de7-d5904ae2bd81",
        "uuid_pub": "30aef9e9-d028-43ce-b100-32115fdb5593",
    },
    "2024-10-01-11-29-55": {
        "uuid": "79f6c6b8-9e48-4c52-b31d-446d0f5de013",
        "uuid_pub": "8806fafc-17fb-49bd-9e61-87ee35f37521",
    },
    "2024-12-09-11-53-11": {
        "uuid": "7a5c5fd5-683d-44a4-a27e-af08ea0229ab",
        "uuid_pub": "a721d65f-14f6-43d2-8f27-41fbbd7063cc",
    },
    "2024-11-04-12-55-59": {
        "uuid": "7b1784ab-8045-4da0-bb69-b1a6c644361d",
        "uuid_pub": "4ef52474-7847-4773-a3d0-d6386c515964",
    },
    "2024-10-29-09-53-44": {
        "uuid": "7b91d33c-2dce-4194-9df3-d6b7ef8950cc",
        "uuid_pub": "90cdb581-f02f-4610-9c21-559bd6a62c74",
    },
    "2024-11-21-16-36-19": {
        "uuid": "7e127b00-dd50-413d-9728-29856c6943b6",
        "uuid_pub": "5bd7c3ab-9aab-460c-8606-4c2764389728",
    },
    "2024-12-09-09-34-43": {
        "uuid": "83588d12-cb89-4034-b4f7-6e13ca74cbce",
        "uuid_pub": "686deaf1-9a5e-4b85-8fcf-8faa3dea0e8d",
    },
    "2024-11-25-16-02-15": {
        "uuid": "8693e856-9347-4295-a139-41ab518a0f6f",
        "uuid_pub": "ecea5699-eb30-4a78-b035-ba3362fc097b",
    },
    "2024-11-03-13-51-43": {
        "uuid": "86d7cf0c-62c5-4ab2-ae90-8288072ba96d",
        "uuid_pub": "b822fd0d-1381-471f-868b-a3b3f81468df",
    },
    "2024-10-01-12-00-49": {
        "uuid": "8a088084-fe3e-43fe-bf2c-498c98a054d8",
        "uuid_pub": "308b20c6-aaed-43d7-bd85-c0f0c1d53d91",
    },
    "2024-11-11-13-06-23": {
        "uuid": "8d5d7e8f-109d-468e-8f52-274d24020b3a",
        "uuid_pub": "dad783fb-d34e-4b0f-9f62-b48fd4dcdb28",
    },
    "2024-11-18-15-46-05": {
        "uuid": "8ef688b4-cde8-4f7d-b366-fce10d11a455",
        "uuid_pub": "4e84005a-763b-4f00-8cc0-c14aaf910071",
    },
    "2024-11-18-13-22-14": {
        "uuid": "93ac4636-de7a-43cf-8057-7f25755f7be9",
        "uuid_pub": "1b164957-ab40-4bc3-8d80-e9b0ff0898e2",
    },
    "2024-11-15-14-14-12": {
        "uuid": "94b2d7c6-5d20-4e9a-9c4a-2e58064faf5c",
        "uuid_pub": "b778d681-1fa9-48b5-a692-f64a4911ba92",
    },
    "2024-11-04-14-19-11": {
        "uuid": "9b10e9db-ab23-4cb2-8487-fa5403d45913",
        "uuid_pub": "9e0b383a-032d-4750-a2f0-f12c3f1ef253",
    },
    "2024-11-02-19-47-44": {
        "uuid": "9c1e0fde-53a3-49f5-a943-0bdcb9c1de4a",
        "uuid_pub": "a103f111-8e04-45fa-8b21-847baf7d2b7b",
    },
    "2024-11-25-16-36-19": {
        "uuid": "a44792cd-b474-4e07-a03d-e19542e78b13",
        "uuid_pub": "2ca7aa04-02ae-4b95-95da-d3db205c901e",
    },
    "2024-11-15-11-18-14": {
        "uuid": "a5499f97-c96e-49fd-a120-5e769f5f00a9",
        "uuid_pub": "febd59b0-b9d1-4e85-8b46-d549317ef20f",
    },
    "2024-11-15-14-43-52": {
        "uuid": "a634dee5-60e5-46e3-a83f-3b062cc42895",
        "uuid_pub": "3e24f7a5-f194-415a-bee4-8162d7de3dcb",
    },
    "2024-11-03-08-42-30": {
        "uuid": "a83081a6-4a07-418a-8cee-943cd024325d",
        "uuid_pub": "2518a42e-8487-44c1-ad94-dafcfece5ab7",
    },
    "2024-10-29-09-08-34": {
        "uuid": "aac088c5-ea62-404e-a21b-59775e8caa9d",
        "uuid_pub": "a2964b97-49ae-43b6-8cf5-0528684d356c",
    },
    "2024-11-15-12-06-03": {
        "uuid": "b08f0bcb-571a-4cfb-9a04-58f0c6928bdd",
        "uuid_pub": "beaa49ce-bcc2-438c-bb3e-78dc0380b435",
    },
    "2024-11-04-14-55-02": {
        "uuid": "b243fc8b-ca51-4950-9659-1368d5385aba",
        "uuid_pub": "368676ff-3b2c-4720-95c0-ba14c7960f42",
    },
    "2024-12-09-11-28-28": {
        "uuid": "b3332086-58ab-43a4-a1e5-a0d63836ac0d",
        "uuid_pub": "47dc0c01-8986-47fb-bfbe-11b61596853b",
    },
    "2024-11-03-11-03-50": {
        "uuid": "b69be373-a126-4a0e-85e9-754fa41ec1da",
        "uuid_pub": "3d9b9a05-fbc2-4fac-8428-0178f6617a9a",
    },
    "2024-11-03-07-52-45": {
        "uuid": "bb6795bd-e38d-4085-bd7b-22248a430f29",
        "uuid_pub": "627fdda9-54ae-4081-8a51-67405fc84789",
    },
    "2024-11-04-11-56-53": {
        "uuid": "be0c6647-3f46-447c-b96b-7816ee33e40d",
        "uuid_pub": "1874ef9a-140a-465b-8de0-4a6516589a9b",
    },
    "2024-11-01-17-46-15": {
        "uuid": "c00eaf5b-42bc-4d6c-a0b2-086e78d16f0c",
        "uuid_pub": "b67d80fc-31ba-46e1-bff9-8a83fa9342aa",
    },
    "2024-11-18-11-42-04": {
        "uuid": "c10798d4-2690-4076-9f70-130b211adee8",
        "uuid_pub": "79940950-ef36-46fd-b66b-6e08fdb83400",
    },
    "2024-11-03-13-59-54": {
        "uuid": "c171bee4-28f4-43c3-827a-ffd7c18583bc",
        "uuid_pub": "f78f6c00-2eec-4e5d-9d4f-2ade4476a8ff",
    },
    "2024-11-14-14-36-02": {
        "uuid": "c513f500-82ad-486c-aebd-6b1ae728ec94",
        "uuid_pub": "bff379f4-e3c4-49ac-9bc1-afd23c2d84c6",
    },
    "2024-11-04-13-07-13": {
        "uuid": "ca6db1b8-f7b4-4498-a0b0-679eb16cef62",
        "uuid_pub": "ca0f3352-86d6-4ebe-a433-804e4332ad86",
    },
    "2024-11-02-17-18-32": {
        "uuid": "cdf8617f-d643-4bb4-9c8f-976c04dafafd",
        "uuid_pub": "8fad8729-1374-4c59-a4eb-2491b4f26bfe",
    },
    "2024-11-15-15-07-36": {
        "uuid": "cf4bdddc-7ed4-444a-8d7a-de59364693eb",
        "uuid_pub": "0522ec59-e693-4052-b285-42fc3efde4c9",
    },
    "2024-11-15-10-16-35": {
        "uuid": "cfa237e4-386d-4539-9ba5-667b1f176a21",
        "uuid_pub": "bf463fae-7b22-4342-8c83-283c8b4b5153",
    },
    "2024-11-15-16-41-14": {
        "uuid": "d66057fc-8e40-4939-a6ce-c4833d2b2964",
        "uuid_pub": "6959420b-a6f8-48e6-bf31-b96df1959eca",
    },
    "2024-11-25-14-57-08": {
        "uuid": "df0abba7-1e61-427f-8111-d1f77d52dddc",
        "uuid_pub": "2d30d658-c94d-42b6-a7a1-b5794b310fbe",
    },
    "2024-11-14-15-22-43": {
        "uuid": "e311107b-1c8e-44dd-abcb-ec60a59ce330",
        "uuid_pub": "2899a1cc-fe37-4555-bab4-a7f130787a94",
    },
    "2024-11-11-12-42-47": {
        "uuid": "e6580214-d883-4a4b-9742-37f44d20e374",
        "uuid_pub": "c593056b-5796-46e4-8471-6208a711f87d",
    },
    "2024-12-03-13-15-38": {
        "uuid": "e6c30062-997a-407d-8a68-232d1f3dd505",
        "uuid_pub": "0579a4e6-3f07-4a41-8a6c-b262bb9ecf9c",
    },
    "2024-11-14-13-45-37": {
        "uuid": "e9a2a009-b893-4743-9d37-eed954469982",
        "uuid_pub": "2bdd6b87-e6ca-4465-beb5-c999551e0bd4",
    },
    "2024-11-02-20-03-21": {
        "uuid": "f4962481-018e-49c2-bd2f-1a570c5c4d11",
        "uuid_pub": "6601f0e5-af17-4369-9f89-a4c2616205f2",
    },
    "test": {"uuid": "f4ab2452-30e0-4727-a8b1-97f7391b20b6", "uuid_pub": "19dbe2dd-dd5b-481a-92eb-52cdc3037301"},
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
    repo: str = "leggedrobotics/grand_tour_box",
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
