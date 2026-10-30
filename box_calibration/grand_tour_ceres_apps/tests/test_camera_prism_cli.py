#!/usr/bin/env python3
"""Synthetic camera/prism CLI checks; all generated bags and results are temporary."""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile

import numpy as np
import rosbag
import rospy
import yaml
from ap20_driver_ros.msg import PositionDebug
from geometry_msgs.msg import Point, PointStamped
from grand_tour_camera_detection_msgs.msg import CameraDetections
from scipy.spatial.transform import Rotation
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", type=Path, required=True)
    args = parser.parse_args()
    results = {}
    with tempfile.TemporaryDirectory(prefix="grand_tour_prism_cli_") as temporary:
        root = Path(temporary)
        env = os.environ.copy()
        env["ROS_HOME"] = str(root / "ros")
        env["OPENBLAS_NUM_THREADS"] = "1"
        calibration = {"camera0": {"rostopic": "/camera0", "frame_id": "camera0",
                       "intrinsics": [600., 610., 640., 480.], "distortion_model": "radtan",
                       "distortion_coeffs": [0., 0., 0., 0.], "resolution": [1280, 960],
                       "T_bundle_camera": np.eye(4).tolist()}}
        (root / "cameras.yaml").write_text(yaml.safe_dump(calibration))
        lever = np.array([0.12, -0.03, 0.07])
        board = np.eye(4)
        board[:3, :3] = Rotation.from_rotvec([0.1, -0.05, 0.2]).as_matrix()
        board[:3, 3] = [0.2, -0.3, 0.1]
        points = np.array([[(x-4.5)*0.045, (y-4.5)*0.045, 0] for y in range(10) for x in range(10)])
        def pose(t):
            result = np.eye(4)
            result[:3, :3] = Rotation.from_euler("xyz", [0.25*np.sin(t), 0.2*np.cos(0.7*t), 0.2*np.sin(1.3*t)]).as_matrix()
            result[:3, 3] = [0.3*np.sin(t), 0.2*np.cos(0.9*t), -2+0.1*np.sin(1.2*t)]
            return result
        with rosbag.Bag(str(root / "camera.bag"), "w") as bag:
            for step in range(1, 120):
                t = step * 0.05
                camera_board = np.linalg.inv(pose(t))
                xyz = points @ camera_board[:3, :3].T + camera_board[:3, 3]
                message = CameraDetections()
                message.header.frame_id = "camera0"
                message.header.stamp = rospy.Time.from_sec(10+t)
                message.cornerids = list(range(len(points)))
                message.modelpoint3d = [Point(*p) for p in points]
                message.corners2d = [Point(600*p[0]/p[2]+640, 610*p[1]/p[2]+480, 0) for p in xyz]
                bag.write("/camera0_corner_detections", message, message.header.stamp)
        def write_prism(path, debug=False, offset=0):
            with rosbag.Bag(str(path), "w") as bag:
                for step in range(1201):
                    t = step * 0.005
                    predicted = board @ pose(t) @ np.r_[lever, 1]
                    message = PositionDebug() if debug else PointStamped()
                    message.header.frame_id = "total_station"
                    message.header.stamp = rospy.Time.from_sec(10+t+offset)
                    if debug:
                        message.position.point = Point(*predicted[:3])
                    else:
                        message.point = Point(*predicted[:3])
                    bag.write("/prism", message, message.header.stamp)
        def run(name, prism, extra=(), good=True):
            output = root / f"{name}.yaml"
            output.write_text("preserve existing")
            command = [str(args.executable), "-c", str(root / "cameras.yaml"),
                       "--camera_bags", str(root / "camera.bag"), "-p", str(prism),
                       "-t", "/prism", "-o", str(output), *map(str, extra)]
            result = subprocess.run(command, capture_output=True, text=True, env=env, timeout=120)
            results[name] = {"exit_code": result.returncode}
            if good:
                assert result.returncode == 0, result.stdout + result.stderr
                actual = np.array(yaml.safe_load(output.read_text())["camera0"]["t_cam_prism"])
                error = float(np.linalg.norm(actual-lever))
                results[name]["translation_error_m"] = error
                assert error < 1e-4, results[name]
            else:
                assert result.returncode != 0, result.stdout + result.stderr
                assert output.read_text() == "preserve existing"
            return output
        write_prism(root / "point.bag")
        run("point_stamped", root / "point.bag", ["--rerun_recording", root / "prism.rrd"])
        results["point_stamped"]["rrd_bytes"] = (root / "prism.rrd").stat().st_size
        assert results["point_stamped"]["rrd_bytes"] > 100
        write_prism(root / "debug.bag", debug=True)
        run("position_debug", root / "debug.bag")
        write_prism(root / "offset.bag", offset=-0.015)
        output = run("negative_time_offset", root / "offset.bag", ["--solve_time_offset"])
        offset = float(re.search(r"time offset \(s\): ([^\n]+)", output.read_text())[1])
        results["negative_time_offset"]["time_error_s"] = abs(offset+0.015)
        assert abs(offset+0.015) < 1e-4
        with rosbag.Bag(str(root / "wrong.bag"), "w") as bag:
            bag.write("/prism", String(data="invalid type"), rospy.Time(10))
        run("wrong_prism_type", root / "wrong.bag", good=False)
        write_prism(root / "outside.bag", offset=100)
        run("no_time_overlap", root / "outside.bag", good=False)
        run("missing_bag", root / "missing.bag", good=False)
        # A solved calibration must still report an output write failure.
        result = subprocess.run([str(args.executable), "-c", str(root / "cameras.yaml"), "--camera_bags",
                                 str(root / "camera.bag"), "-p", str(root / "point.bag"), "-t", "/prism",
                                 "-o", str(root / "absent/output.yaml")], env=env, capture_output=True, timeout=120)
        assert result.returncode != 0
        results["output_failure"] = {"exit_code": result.returncode}
    results["temporary_outputs_removed"] = not root.exists()
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
