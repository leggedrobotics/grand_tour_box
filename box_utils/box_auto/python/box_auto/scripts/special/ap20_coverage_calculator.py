#!/usr/bin/env python3
# file: automate_bag_path_length.py
#
# Computes total DLIO odometry path length (meters) per mission and, in addition,
# computes Coverage (%) of AP20 ground-truth position availability over the walked distance.
#
# Output YAML schema (list of rows):
# - abbreviated: <str>
#   distance_m: <float>
#   coverage_pct: <float|null>

from __future__ import annotations
from box_auto.utils.trajectory_statistics import input_provenance, append_summary
from box_auto.utils.trajectory_statistics import (
    compute_odom_edges_and_length,
    read_ap20_timestamps,
    build_intervals_from_times,
    covered_distance_from_edges,
)

import os
import shutil
import tempfile
from pathlib import Path
from typing import Iterable

# --- third-party / project utilities ---
from box_auto.utils import get_uuid_mapping, read_sheet_data, get_bag
import kleinkram

# # --- optional YAML dependency (preferred) ---
# try:

# except Exception:
#     yaml = None


# -------------------------
# Config
# -------------------------
SPREADSHEET_ID = "1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg"

# DLIO odometry topic (primary)
ODOM_TOPICS_CANDIDATES = [
    "/boxi/dlio/lidar_map_odometry",
]

# AP20 (prism) topic (ground truth availability)
AP20_TOPIC = "/boxi/ap20/prism_position"

# Remote bag name patterns
ODOM_BAG_PATTERNS = ["*_dlio.bag"]
AP20_BAG_PATTERNS = ["*_ap20_prism_position.bag"]

# Skip first N missions
SKIP_FIRST = 0

# Minimum file size (MiB) to consider a bag "non-empty"
MIN_BAG_MIB_ODO = 0.1
MIN_BAG_MIB_AP20 = 0.05

# Where to fetch/process and where to write the summary
TMP_BASE = Path(os.getenv("TMP_BASE", "/tmp"))
OUTPUT_FILE = Path(os.getenv("ODOM_SUMMARY_YAML", "coverage_odom_lengths.yaml"))


# -------------------------
# Helpers
# -------------------------


def select_best_remote_bag(mission_uuid_release: str, patterns: list[str], min_mib: float) -> str | None:
    """Return the filename of the largest candidate bag for the mission matching `patterns`."""
    candidates = kleinkram.list_files(mission_ids=[mission_uuid_release], file_names=patterns)
    candidates = [f for f in candidates if round(f.size / 1024 / 1024, 3) > min_mib]
    if not candidates:
        return None
    candidates.sort(key=lambda f: f.size, reverse=True)
    return candidates[0].file_name if hasattr(candidates[0], "file_name") else candidates[0].name


def download_remote_files(mission_uuid_release: str, file_patterns: Iterable[str], dest: Path) -> None:
    kleinkram.download(
        mission_ids=[mission_uuid_release],
        file_names=list(file_patterns),
        dest=dest,
        verbose=True,
    )


def find_local_file(pattern_or_name: str, directory: Path) -> Path:
    """Return a single local file path matching pattern; raises if not found or ambiguous."""
    p = get_bag(pattern_or_name, directory=directory)
    return Path(p)


def append_result_yaml(out_path, abbreviated, distance_m, coverage_pct=None, provenance=None):
    row = {"abbreviated": abbreviated or "", "distance_m": float(distance_m), "provenance": provenance}
    row["coverage_pct"] = None if coverage_pct is None else float(coverage_pct)
    row["availability_policy"] = "gap=min(0.5s,3*median_dt); padding=median_dt/2; uniform edge speed"
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

        # --- choose bags (largest by size) ---
        remote_odom_name = select_best_remote_bag(uuid_release, ODOM_BAG_PATTERNS, MIN_BAG_MIB_ODO)
        if remote_odom_name is None:
            print(f"MISSION: {name} has no DLIO bag; skipping")
            continue

        remote_ap20_name = select_best_remote_bag(uuid_release, AP20_BAG_PATTERNS, MIN_BAG_MIB_AP20)
        if remote_ap20_name is None:
            print(f"MISSION: {name} has no AP20 bag; coverage will be null")

        TMP_BASE.mkdir(parents=True, exist_ok=True)
        tmp_folder = Path(tempfile.mkdtemp(prefix="box_auto_download_", dir=TMP_BASE))
        try:
            tmp_folder.mkdir(parents=True, exist_ok=True)

            # --- download required bags ---
            to_fetch = [remote_odom_name] + ([remote_ap20_name] if remote_ap20_name else [])
            print(f"MISSION: {name} -> downloading: {', '.join(to_fetch)}")
            download_remote_files(uuid_release, to_fetch, tmp_folder)

            # --- resolve local paths ---
            local_odom = find_local_file(remote_odom_name, tmp_folder)
            if not local_odom.exists():
                local_odom = find_local_file(Path(remote_odom_name).name, tmp_folder)

            local_ap20 = None
            if remote_ap20_name:
                try:
                    local_ap20 = find_local_file(remote_ap20_name, tmp_folder)
                except Exception:
                    local_ap20 = find_local_file(Path(remote_ap20_name).name, tmp_folder)

            # --- compute odom total + edges ---
            total_m, edges = compute_odom_edges_and_length(local_odom, ODOM_TOPICS_CANDIDATES[0])
            if total_m is None or not edges:
                print(f"MISSION: {name} -> no messages on DLIO odom topic; skipping")
                continue

            # --- compute coverage ---
            coverage_pct = None
            if local_ap20 and Path(local_ap20).exists():
                ap20_times = read_ap20_timestamps(local_ap20, AP20_TOPIC)
                gt_intervals = build_intervals_from_times(ap20_times)
                if gt_intervals and total_m > 0.0:
                    d_cov = covered_distance_from_edges(edges, gt_intervals)
                    coverage_pct = round(100.0 * d_cov / total_m, 3)
                else:
                    coverage_pct = None  # not enough GT samples / no intervals
            else:
                coverage_pct = None  # no AP20 bag

            abbrev = MISSION_DATA[name].get("abbreviated")

            append_result_yaml(
                OUTPUT_FILE,
                abbrev,
                total_m,
                coverage_pct,
                provenance={
                    "odometry": input_provenance(local_odom, ODOM_TOPICS_CANDIDATES[0]),
                    "ap20": input_provenance(local_ap20, AP20_TOPIC) if local_ap20 else None,
                },
            )
            cov_str = "null" if coverage_pct is None else f"{coverage_pct:.3f}%"
            print(f"Appended: {abbrev} -> {total_m:.3f} m, coverage: {cov_str}")
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
    parser.add_argument("--ap20-bag", type=Path)
    parser.add_argument("--ap20-topic", default=AP20_TOPIC)
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
            if args.ap20_bag:
                times = read_ap20_timestamps(args.ap20_bag, args.ap20_topic)
                intervals = build_intervals_from_times(times)
                if intervals and total > 0:
                    coverage = 100.0 * covered_distance_from_edges(edges, intervals) / total
                provenance["ap20"] = input_provenance(args.ap20_bag, args.ap20_topic)
            append_result_yaml(pending, args.label, total, coverage, provenance)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
