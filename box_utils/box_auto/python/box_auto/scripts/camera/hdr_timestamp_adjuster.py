#!/usr/bin/env python3
"""Compatibility entry point for legacy *_updated.bag consumers.

Reuse the combined processor's validated timing and metadata handling. Keeping
this output suffix permits existing upload/migration scripts to run unchanged.
"""
import argparse
from combined_hdr_camera_processing import RosbagValidatorAndProcessor


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--skip_update", action="store_true")
    args = parser.parse_args(argv)
    RosbagValidatorAndProcessor(
        ["*hdr_front.bag", "*hdr_left.bag", "*hdr_right.bag"], "_updated", delete_original=False
    ).run(skip_update=args.skip_update)


if __name__ == "__main__":
    main()
