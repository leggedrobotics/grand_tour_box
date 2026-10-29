#!/usr/bin/env python3
"""Save the layout for RerunCameraImuViz (rerun-sdk 0.30, Python 3.10+)."""

import argparse

import rerun as rr
import rerun.blueprint as rrb


def build_blueprint() -> rrb.Blueprint:
    # Entity paths and timeline match rerun_camera_imu_viz.cpp/.h.
    scene = rrb.Spatial3DView(name="Camera and IMU poses", origin="world")
    motion = rrb.Vertical(
        rrb.TimeSeriesView(name="Position (m)", origin="world/imu/position"),
        rrb.TimeSeriesView(name="Velocity (m/s)", origin="world/imu/velocity"),
        rrb.TimeSeriesView(name="Orientation (rad)", origin="world/imu/orientation"),
    )
    diagnostics = rrb.Vertical(
        rrb.TimeSeriesView(name="Accelerometer bias (m/s²)", origin="world/imu/acc_bias"),
        rrb.TimeSeriesView(name="Gyroscope bias (rad/s)", origin="world/imu/gyro_bias"),
        rrb.TimeSeriesView(name="IMU consistency residuals", origin="world/imu/consistency"),
        rrb.TimeSeriesView(name="Camera and IMU trajectories", origin="imu_trajectory"),
    )
    return rrb.Blueprint(
        rrb.Horizontal(scene, motion, diagnostics),
        rrb.TimePanel(timeline="camera_imu_timeline"),
        collapse_panels=True,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", "-o", default="camera_imu.rbl", help="Output .rbl file")
    args = parser.parse_args()
    rr.init("camera_imu_offline_calibration", spawn=False)
    build_blueprint().save("camera_imu_offline_calibration", args.output)
    print(f"Blueprint saved to: {args.output}")
