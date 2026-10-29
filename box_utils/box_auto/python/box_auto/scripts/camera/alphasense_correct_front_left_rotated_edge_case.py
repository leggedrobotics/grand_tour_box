#!/usr/bin/env python3
"""Repair inverted images whose CameraInfo already describes the intended orientation.

This preserves the sensor's calibration and exposure times. It is not a general
rotation into a new optical frame: use it only for the inverted-image recording
fault, for which the final pixel orientation matches the existing CameraInfo.
"""

import argparse
import copy
from pathlib import Path
import sys

import cv2
import numpy as np
import rosbag
from sensor_msgs.msg import CompressedImage

from box_auto.utils.atomic_file import atomic_output

TARGET_TOPIC = "/boxi/alphasense/front_left/image_raw/compressed"
JPEG_QUALITY = 95


def guess_imencode_ext(compressed_format):
    fmt = (compressed_format or "").lower()
    if "compresseddepth" in fmt:
        raise ValueError("This image repair does not support compressedDepth")
    if "png" in fmt:
        return ".png"
    if "jpg" in fmt or "jpeg" in fmt:
        return ".jpg"
    raise ValueError(f"Unsupported image format: {compressed_format!r}")


def rotate_compressed_image_180(msg):
    if getattr(msg, "_type", None) != "sensor_msgs/CompressedImage":
        raise ValueError("Expected sensor_msgs/CompressedImage")
    ext = guess_imencode_ext(msg.format)
    image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise ValueError("Cannot decode image selected for rotation")
    params = [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY] if ext == ".jpg" else []
    ok, encoded = cv2.imencode(ext, cv2.rotate(image, cv2.ROTATE_180), params)
    if not ok:
        raise ValueError("Cannot encode rotated image")
    output = CompressedImage()
    output.header = copy.deepcopy(msg.header)
    output.format = msg.format
    output.data = encoded.tobytes()
    return output


def rotate_bag(input_bag, output_bag, topic=TARGET_TOPIC, *, overwrite=False):
    if Path(input_bag).resolve() == Path(output_bag).resolve():
        raise ValueError("Rotation output must differ from its input")
    rotated = 0
    with atomic_output(output_bag, overwrite=overwrite) as temporary:
        with rosbag.Bag(str(input_bag), "r") as source, rosbag.Bag(str(temporary), "w", compression="lz4") as output:
            for message_topic, msg, stamp in source.read_messages():
                if message_topic == topic:
                    msg = rotate_compressed_image_180(msg)
                    rotated += 1
                # Preserve CameraInfo, timestamps and every unrelated stream.
                output.write(message_topic, msg, stamp)
            if not rotated:
                raise ValueError(f"No images on {topic}; existing output is preserved")
    return rotated


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--topic", default=TARGET_TOPIC)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args(argv)
    output = args.output or args.input.with_name(args.input.stem + "_rotated.bag")
    try:
        count = rotate_bag(args.input, output, args.topic, overwrite=args.overwrite)
    except Exception as exc:
        print(f"Rotation failed: {exc}", file=sys.stderr)
        return 1
    print(f"Rotated {count} images: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
