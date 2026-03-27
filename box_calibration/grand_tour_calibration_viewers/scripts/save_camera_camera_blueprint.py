#!/usr/bin/env python3
"""
Build and save the camera-camera calibration Rerun blueprint to a .rbl file.

The saved file can be loaded in C++ via:
    rec.log_file_from_path("/path/to/camera_camera.rbl");

Usage:
    python3 save_camera_camera_blueprint.py [--output /path/to/camera_camera.rbl]
"""
import argparse
import rerun as rr
import rerun.blueprint as rrb

BASE = "world/"


def cam_view(name: str, cam: str) -> rrb.Spatial2DView:
    return rrb.Spatial2DView(name=name, origin=BASE + cam)


def res_view(name: str, cam: str) -> rrb.Spatial2DView:
    return rrb.Spatial2DView(name=name, origin=BASE + cam + "_residuals", background=[255, 255, 255])


def build_blueprint() -> rrb.Blueprint:
    detections_panel = rrb.Vertical(
        rrb.Horizontal(
            rrb.Vertical(
                cam_view("Alphasense Left", "cam4_sensor_frame"),
                cam_view("HDR Left",        "hdr_left"),
            ),
            rrb.Vertical(
                rrb.Horizontal(
                    cam_view("Alphasense Front Left",   "cam3_sensor_frame"),
                    cam_view("Alphasense Front Centre", "cam1_sensor_frame"),
                    cam_view("Alphasense Front Right",  "cam2_sensor_frame"),
                ),
                cam_view("HDR Front", "hdr_front"),
                rrb.Horizontal(
                    cam_view("ZED Left",  "zed2i_left_camera_optical_frame"),
                    cam_view("ZED Right", "zed2i_right_camera_optical_frame"),
                ),
            ),
            rrb.Vertical(
                cam_view("Alphasense Right", "cam5_sensor_frame"),
                cam_view("HDR Right",        "hdr_right"),
            ),
            column_shares=[1, 2, 1],
            name="Camera Detections",
        ),
        rrb.Horizontal(
            rrb.Vertical(
                res_view("Alphasense Left Residuals", "cam4_sensor_frame"),
                res_view("HDR Left Residuals",        "hdr_left"),
            ),
            rrb.Vertical(
                rrb.Horizontal(
                    res_view("Alphasense Front Left Residuals",   "cam3_sensor_frame"),
                    res_view("Alphasense Front Centre Residuals", "cam1_sensor_frame"),
                    res_view("Alphasense Front Right Residuals",  "cam2_sensor_frame"),
                ),
                res_view("HDR Front Residuals", "hdr_front"),
                rrb.Horizontal(
                    res_view("ZED Left Residuals",  "zed2i_left_camera_optical_frame"),
                    res_view("ZED Right Residuals", "zed2i_right_camera_optical_frame"),
                ),
            ),
            rrb.Vertical(
                res_view("Alphasense Right Residuals", "cam5_sensor_frame"),
                res_view("HDR Right Residuals",        "hdr_right"),
            ),
            column_shares=[1, 2, 1],
            name="Intrinsics Reprojection Error",
        ),
    )

    top_row = rrb.Horizontal(
        detections_panel,
        rrb.Spatial3DView(name="Extrinsics", origin=BASE),
        column_shares=[2, 1],
    )

    stats_row = rrb.Horizontal(
        # rrb.GraphView(name="Camera Adjacency",    origin=BASE + "adjacency"),
        rrb.TextDocumentView(name="Calibration Sigmas", origin=BASE + "calibration_sigmas"),
        name="Camera Optimisation Stats",
    )

    layout = rrb.Horizontal(
        rrb.BarChartView(name="Data Collection Progress", origin=BASE + "data_accumulation_progress"),
        # rrb.Vertical(
            top_row,
            rrb.TextDocumentView(name="Calibration Sigmas", origin=BASE + "calibration_sigmas"),
        #     stats_row,
        #     row_shares=[2, 1],
        # ),
        column_shares=[0.3, 8, 2],
    )

    return rrb.Blueprint(layout, collapse_panels=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Save camera-camera calibration Rerun blueprint.")
    parser.add_argument(
        "--output", "-o",
        default="/home/fu/github/grand_tour_box/box_calibration/grand_tour_ceres_apps/config/camera_camera.rbl",
        help="Output .rbl file path (default: camera_camera.rbl)",
    )
    args = parser.parse_args()

    rr.init("rerun_blueprint_example", spawn=True)
    blueprint = build_blueprint()
    rr.send_blueprint(blueprint)
    rr.send_blueprint(blueprint)
    blueprint.save("camera_camera_calibration", args.output)
    print(f"Blueprint saved to: {args.output}")
    print(f"Load in C++ with: rec.log_file_from_path(\"{args.output}\");")
