#!/usr/bin/env python3
"""Compatibility encoder for already timestamp-adjusted HDR bags."""
from box_auto.utils import get_bag, upload_bag
from combined_hdr_camera_processing import RosbagValidatorAndProcessor


def run(input_bag_path, output_bag_path, camera):
    # The shared encoder preserves record time for headerless/zero-stamp metadata.
    RosbagValidatorAndProcessor([], "_encoding").encode_only_bag(input_bag_path, output_bag_path, camera)


def main():
    for camera in ("hdr_front", "hdr_left", "hdr_right"):
        source = get_bag(f"*{camera}_updated.bag")
        output = source[: -len("_updated.bag")] + "_encoding.bag"
        run(source, output, camera)
        upload_bag(output)


if __name__ == "__main__":
    main()
