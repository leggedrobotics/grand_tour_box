import os
from pathlib import Path
import psutil
from time import sleep

WS = "/home/catkin_ws"
PRE = f"source /opt/ros/noetic/setup.bash; source {WS}/devel/setup.bash; "
MISSION_DATA = os.environ.get("MISSION_DATA", "/mission_data")
BOX_AUTO_SCRIPTS_DIR = str(Path(__file__).parent.parent / "scripts")
BOX_AUTO_DIR = str(Path(__file__).parent.parent.parent.parent)


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
