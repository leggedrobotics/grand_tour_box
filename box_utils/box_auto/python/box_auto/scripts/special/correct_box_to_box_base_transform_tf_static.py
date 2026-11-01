#!/usr/bin/env python3
"""Apply an explicit base-to-box calibration without accumulating TF children."""

import argparse
import copy
from pathlib import Path

import numpy as np
import rosbag
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation

from box_auto.utils.atomic_file import atomic_output

# Historical mission-specific correction, distinct from the nominal URDF.
# Its provenance has not been established for other missions; select it explicitly.
LEGACY_XYZ_RPY = (0.0764038, -0.0361, 0.2802790, 3.1415126, 0.0, 0.030543)


def repair_bag(input_bag, output_bag, xyz_rpy, *, overwrite=False):
    values = np.asarray(xyz_rpy, dtype=float)
    if values.shape != (6,) or not np.isfinite(values).all():
        raise ValueError("Calibration must contain six finite values: x y z roll pitch yaw (m, rad)")
    rotation = Rotation.from_euler("xyz", values[3:]).as_quat()
    replaced = 0
    with atomic_output(output_bag, overwrite=overwrite) as temporary:
        with rosbag.Bag(str(input_bag), "r") as source, rosbag.Bag(str(temporary), "w", compression="lz4") as output:
            for topic, message, stamp in source.read_messages():
                if topic == "/tf_static":
                    message = copy.deepcopy(message)
                    transforms = []
                    base_transform = None
                    spider_transform = None
                    for transform in message.transforms:
                        if transform.child_frame_id == "box_base":
                            if transform.header.frame_id != "base":
                                raise ValueError("box_base has an unexpected parent; calibration is not applicable")
                            if base_transform is not None:
                                continue
                            base_transform = transform
                            t, q = transform.transform.translation, transform.transform.rotation
                            t.x, t.y, t.z = values[:3]
                            q.x, q.y, q.z, q.w = rotation
                            replaced += 1
                        elif transform.child_frame_id == "spider":
                            if transform.header.frame_id != "box_base":
                                raise ValueError("spider has an unexpected parent; refusing to change the TF tree")
                            if spider_transform is not None:
                                continue
                            spider_transform = transform
                            # This repair defines spider at the box_base origin.
                            transform.transform = TransformStamped().transform
                            transform.transform.rotation.w = 1.0
                        transforms.append(transform)
                    if base_transform is not None and spider_transform is None:
                        spider = TransformStamped()
                        spider.header = copy.deepcopy(base_transform.header)
                        spider.header.frame_id = "box_base"
                        spider.child_frame_id = "spider"
                        spider.transform.rotation.w = 1.0
                        transforms.append(spider)
                    # Repeated snapshots are valid; duplicate children within a
                    # snapshot are not. Re-running this repair preserves the tree.
                    message.transforms = transforms
                output.write(topic, message, stamp)
            if not replaced:
                raise ValueError("No base -> box_base transform found; existing output is preserved")
    return replaced


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    destinations = parser.add_mutually_exclusive_group()
    destinations.add_argument("--output", type=Path)
    destinations.add_argument("--in-place", action="store_true")
    calibration = parser.add_mutually_exclusive_group(required=True)
    calibration.add_argument("--xyz-rpy", nargs=6, type=float, metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"))
    calibration.add_argument(
        "--legacy-calibration", action="store_true", help="Use the historical mission-specific correction"
    )
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args(argv)
    output = (
        args.input if args.in_place else (args.output or args.input.with_name(args.input.stem + "_recalibrated.bag"))
    )
    if not args.in_place and output.resolve() == args.input.resolve():
        parser.error("Use --in-place to replace the input after successful validation")
    count = repair_bag(
        args.input,
        output,
        args.xyz_rpy if args.xyz_rpy is not None else LEGACY_XYZ_RPY,
        overwrite=args.in_place or args.overwrite,
    )
    print(f"Replaced {count} calibration snapshots: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
