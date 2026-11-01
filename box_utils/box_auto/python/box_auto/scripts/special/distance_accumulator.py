#!/usr/bin/env python3
# file: automate_bag_path_length.py

from __future__ import annotations
from box_auto.utils.trajectory_statistics import input_provenance, append_summary
from box_auto.utils.trajectory_statistics import (
    compute_odom_edges_and_length,
)

import os
import shutil
import tempfile
from pathlib import Path

# --- third-party / project utilities ---
from box_auto.utils import get_uuid_mapping, read_sheet_data, get_bag
import kleinkram

# --- optional YAML dependency (preferred) ---
try:
    import yaml  # PyYAML
except Exception:
    yaml = None


# -------------------------
# Config
# -------------------------
SPREADSHEET_ID = "1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg"

# Only DLIO odometry topic (primary)
ODOM_TOPICS_CANDIDATES = [
    "/boxi/dlio/lidar_map_odometry",
]

# Only DLIO bags
ODOM_BAG_PATTERNS = [
    "*_dlio.bag",
]

# Skip first N missions
SKIP_FIRST = 0

# Minimum file size (MiB) to consider a bag "non-empty"
MIN_BAG_MIB = 0.1

# Where to fetch/process and where to write the summary
TMP_BASE = Path(os.getenv("TMP_BASE", "/tmp"))
OUTPUT_FILE = Path(os.getenv("ODOM_SUMMARY_YAML", "odom_lengths.yaml"))


# -------------------------
# Helpers
# -------------------------


def compute_path_length_m(bag_path, topic):
    return compute_odom_edges_and_length(bag_path, topic)[0]


def select_best_remote_bag(mission_uuid_release: str) -> str | None:
    """Return the filename of the largest candidate DLIO bag for the mission."""
    candidates = kleinkram.list_files(mission_ids=[mission_uuid_release], file_names=ODOM_BAG_PATTERNS)
    candidates = [f for f in candidates if round(f.size / 1024 / 1024, 3) > MIN_BAG_MIB]
    if not candidates:
        return None
    candidates.sort(key=lambda f: f.size, reverse=True)
    return candidates[0].file_name if hasattr(candidates[0], "file_name") else candidates[0].name


def download_remote_files(mission_uuid_release: str, file_patterns: list[str], dest: Path) -> None:
    kleinkram.download(
        mission_ids=[mission_uuid_release],
        file_names=file_patterns,
        dest=dest,
        verbose=True,
    )


def find_local_file(pattern: str, directory: Path) -> Path:
    """Return a single local file path matching pattern; raises if not found or ambiguous."""
    p = get_bag(pattern, directory=directory)
    return Path(p)


def append_result_yaml(out_path, abbreviated, distance_m, coverage_pct=None, provenance=None):
    row = {"abbreviated": abbreviated or "", "distance_m": float(distance_m), "provenance": provenance}
    append_summary(out_path, row)


# -------------------------
# Main automation
# -------------------------
def run_batch():
    failures = 0
    uuid_mappings = get_uuid_mapping()
    _, MISSION_DATA = read_sheet_data(SPREADSHEET_ID)

    # Clear existing YAML file if present
    OUTPUT_FILE.parent.mkdir(parents=True, exist_ok=True)
    if OUTPUT_FILE.exists():
        OUTPUT_FILE.write_text("")  # truncate to empty
    print(f"YAML file (cleared/ready): {OUTPUT_FILE.resolve()}")

    for j, (name, data) in enumerate(uuid_mappings.items(), start=1):
        if j <= SKIP_FIRST:
            continue

        if name not in MISSION_DATA:
            print("Mission not found in sheet:", name)
            continue

        if MISSION_DATA[name].get("GOOD_MISSION") != "TRUE":
            print(f"Skipping non-good mission: {name}")
            continue

        uuid_release = data["uuid_release"]

        remote_bag_name = select_best_remote_bag(uuid_release)
        if remote_bag_name is None:
            print(f"MISSION: {name} has no DLIO bag; skipping")
            continue

        TMP_BASE.mkdir(parents=True, exist_ok=True)
        tmp_folder = Path(tempfile.mkdtemp(prefix="box_auto_download_", dir=TMP_BASE))
        try:
            tmp_folder.mkdir(parents=True, exist_ok=True)

            print(f"MISSION: {name} -> downloading {remote_bag_name}")
            download_remote_files(uuid_release, [remote_bag_name], tmp_folder)

            local_bag = find_local_file(remote_bag_name, tmp_folder)
            if not local_bag.exists():
                local_bag = find_local_file(Path(remote_bag_name).name, tmp_folder)

            meters = None
            for topic in ODOM_TOPICS_CANDIDATES:
                meters = compute_path_length_m(local_bag, topic)
                if meters is not None:
                    break

            if meters is None:
                print(f"MISSION: {name} -> no messages on DLIO odom topic; skipping")
                continue

            abbrev = MISSION_DATA[name].get("abbreviated")

            append_result_yaml(OUTPUT_FILE, abbrev, meters, provenance=input_provenance(local_bag, topic))
            print(f"Appended: {abbrev} -> {meters:.3f} m")
            print(f"YAML file: {OUTPUT_FILE.resolve()}")

        except Exception as e:
            failures += 1
            print("Error:", e)
        finally:
            shutil.rmtree(tmp_folder, ignore_errors=True)

    return 1 if failures else 0


def main(argv=None):
    import argparse
    from box_auto.utils.atomic_file import atomic_output

    global OUTPUT_FILE
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--bag", type=Path, help="Local odometry bag")
    mode.add_argument("--batch", action="store_true", help="Read remote mission metadata and download bags")
    parser.add_argument("--topic", default=ODOM_TOPICS_CANDIDATES[0])
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--label", default="local")
    args = parser.parse_args(argv)
    with atomic_output(args.output, overwrite=False) as pending:
        if args.batch:
            OUTPUT_FILE = pending
            status = run_batch()
            if status or not pending.is_file() or pending.stat().st_size == 0:
                raise RuntimeError("Batch failed or produced no results; output was not published")
        else:
            total, edges = compute_odom_edges_and_length(args.bag, args.topic)
            if total is None or not edges:
                raise ValueError("At least two odometry samples are required")
            coverage = None
            provenance = {"odometry": input_provenance(args.bag, args.topic)}
            append_result_yaml(pending, args.label, total, coverage, provenance)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
