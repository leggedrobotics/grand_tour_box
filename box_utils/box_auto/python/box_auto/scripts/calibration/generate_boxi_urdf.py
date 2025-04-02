#!/usr/bin/env python3

from time import sleep
from box_auto.utils import (
    WS,
    kill_roscore,
    run_ros_command,
)
from pathlib import Path

if __name__ == "__main__":

    expected_urdf_path = Path(WS) / "src/grand_tour_box/box_model/box_model/urdf/box/boxi.urdf"
    xacro_path = Path(WS) / "src/grand_tour_box/box_model/box_model/urdf/box/box.urdf.xacro"

    kill_roscore()
    run_ros_command(
        f"roslaunch box_model generate_boxi_urdf.launch urdf_file:={expected_urdf_path} xacro_file:={xacro_path}",
        background=True,
    )

    sleep(2)
    kill_roscore()

    # Check if the expected URDF file exists
    if expected_urdf_path.exists():
        print(f"Successfully generated URDF file at: {expected_urdf_path}")
    else:
        print(f"Failed to generate URDF file at: {expected_urdf_path}")
        raise FileNotFoundError(f"Expected URDF file not found: {expected_urdf_path}")
