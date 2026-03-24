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


def cam_view(name: str, axis: str) -> rrb.TimeSeriesView:
    return rrb.TimeSeriesView(name=name, origin=BASE + axis)


def res_view(name: str, plane: str) -> rrb.Spatial2DView:
    return rrb.Spatial2DView(name=name, origin=BASE + plane + "_residuals", background=[255, 255, 255])


def build_blueprint() -> rrb.Blueprint:
    alignment_panel = rrb.Vertical(
        rrb.Horizontal(
            cam_view("Prism Position X (m)", "prism/x"),
            cam_view("Prism Position Y (m)", "prism/y"),
            cam_view("Prism Position Z (m)", "prism/z"),
            name="Prism Observations and Estimates"
        ),
        rrb.Horizontal(
            res_view("Prism Residuals XY (mm)", "XY"),
            res_view("Prism Residuals YZ (mm)", "YZ"),
            res_view("Prism Residuals XZ (mm)", "XZ"),
            name="Prism Alignment Residuals"
        )
    )

    left_column = rrb.Horizontal(
        alignment_panel,
        rrb.Spatial3DView(
            origin=BASE,
            name="3D Scene",
            # Configure the line grid.
            line_grid=rrb.LineGrid3D(
                visible=True,  # The grid is enabled by default, but you can hide it with this property.
                spacing=1.0,  # Makes the grid more fine-grained.
                # By default, the plane is inferred from view coordinates setup, but you can set arbitrary planes.
                plane=rr.components.Plane3D.XY.with_distance(0.0),
                stroke_width=1.0,  # Makes the grid lines twice as thick as usual.
                color=[255, 255, 255, 128],  # Colors the grid a half-transparent white.
            ),
            spatial_information=rrb.SpatialInformation(
                target_frame="tf#/",
                show_axes=True,
                show_bounding_box=False,
            ),
        ),
        column_shares=[2, 1],
        name="Prism Calibration"
    )

    layout = rrb.Horizontal(
        left_column,
        rrb.TextDocumentView(name="Calibration Results", origin=BASE + "calibration_results"),
        column_shares=[8, 2],
    )

    return rrb.Blueprint(layout, rr.blueprint.TimePanel(timeline="prism_data_timeline"), collapse_panels=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Save camera-prism calibration Rerun blueprint.")
    parser.add_argument(
        "--output", "-o",
        default="/home/fu/github/grand_tour_box/box_calibration/grand_tour_ceres_apps/config/camera_prism.rbl",
        help="Output .rbl file path (default: camera_camera.rbl)",
    )
    args = parser.parse_args()

    rr.init("rerun_blueprint_example", spawn=True)
    blueprint = build_blueprint()
    rr.send_blueprint(blueprint)
    blueprint.save("camera_camera_calibration", args.output)
    print(f"Blueprint saved to: {args.output}")
    print(f"Load in C++ with: rec.log_file_from_path(\"{args.output}\");")
