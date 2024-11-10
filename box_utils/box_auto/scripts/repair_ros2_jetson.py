import argparse
from pathlib import Path
import os
import shutil

MISSION_DATA = os.environ.get("MISSION_DATA", "/mission_data")
import subprocess


def main(overwrite: bool):
    # Get all .bag.active files
    bag_files = [s for s in Path(MISSION_DATA).rglob("*.mcap") if (str(s).find(".orig.") == -1)]

    for p in bag_files:
        # Step 1: Check if bag file is invalid
        check_cmd = f'/data/workspaces/ros2_ws/src/isaac_ros_common/scripts/run_recording.sh "/bin/mcap info {p}"'
        result = subprocess.run(check_cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        print(result.stdout, result.stderr)
        if "Invalid magic at end of file" in result.stdout:
            # Step 2: Copy the original bag file
            orig_file = p.with_suffix(".orig.mcap")
            shutil.copy2(p, orig_file)

            # Step 3: Recover the bag file
            recover_cmd = f'/data/workspaces/ros2_ws/src/isaac_ros_common/scripts/run_recording.sh "/bin/mcap recover {orig_file} -o {p}"'
            os.system(recover_cmd)
        else:
            print(f"Bag file {p} is valid. Skipping...")

        cmd = f'/data/workspaces/ros2_ws/src/isaac_ros_common/scripts/run_recording.sh "source /opt/ros/humble/setup.bash; ros2 bag reindex -s mcap  {p.parent} " '
        os.system(cmd)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Fix and reindex ROS bag files.")
    parser.add_argument(
        "--overwrite", type=bool, default=True, help="Whether to overwrite existing bag files (default: True)."
    )
    args = parser.parse_args()
    # Run main function with the parsed argument
    main(args.overwrite)
