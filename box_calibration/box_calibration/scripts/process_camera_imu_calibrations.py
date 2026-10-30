#!/usr/bin/env python3
"""Calibrate available configured IMUs using existing detections or decoded image bags."""
import argparse
from pathlib import Path
import rospkg
from box_calibration.camera_imu_batch import run_camera_imu_batch, topic_overrides



def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder")
    parser.add_argument("--calibration", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--imu-topic", action="append", default=[])
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    allan = Path(rospkg.RosPack().get_path("box_calibration")) / "calibration/allan_variances"
    run_camera_imu_batch(args.folder, args.calibration, args.output, allan,
                         topic_overrides(args.imu_topic), args.dry_run)


if __name__ == "__main__":
    main()
