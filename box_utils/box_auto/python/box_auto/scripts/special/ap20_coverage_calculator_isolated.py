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
from box_auto.utils.trajectory_statistics import (
    compute_odom_edges_and_length,
    read_ap20_timestamps,
    build_intervals_from_times,
    covered_distance_from_edges,
)

import os
import math
import shutil
import warnings
import statistics
import argparse
import fnmatch
from pathlib import Path
from typing import Iterable

# # --- optional YAML dependency (preferred) ---
# try:
import yaml  # PyYAML

# except Exception:
#     yaml = None

# --- ROS bag import (suppress SyntaxWarnings from rosbag import paths) ---
with warnings.catch_warnings():
    warnings.simplefilter("ignore", SyntaxWarning)
    import rosbag  # ROS1 bag reader

from nav_msgs.msg import Odometry  # ensure message defs are available
from geometry_msgs.msg import PointStamped  # ensure message defs are available


# -------------------------
# Config
# -------------------------
# Base directory containing mission bag(s)
BASE_DIR = Path(os.getenv("BAG_DIR", "/home/tutuna/ijrr_paper/src/test_tests/con_4")).expanduser().resolve()

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

# AP20 acceptance threshold: accept anything bigger than 10 KiB
MIN_AP20_BYTES = 10 * 1024
MIN_BAG_MIB_AP20 = MIN_AP20_BYTES / 1024.0 / 1024.0

# Where to write the summary (unused now, kept as-is)
OUTPUT_FILE = Path(os.getenv("ODOM_SUMMARY_YAML", "coverage_odom_lengths.yaml"))


# -------------------------
# Helpers
# -------------------------


def _glob_candidates(directory: Path, patterns: list[str]) -> list[Path]:
    # Non-recursive: only consider files directly in `directory` (no subdirectories).
    out: list[Path] = []
    for p in directory.iterdir():
        if not p.is_file():
            continue
        for pat in patterns:
            if fnmatch.fnmatch(p.name, pat):
                out.append(p)
                break
    return out


def select_best_local_bag(directory: Path, patterns: list[str], min_mib: float) -> Path | None:
    """Return the path of the largest candidate bag in `directory` matching `patterns`."""
    candidates = _glob_candidates(directory, patterns)
    candidates = [p for p in candidates if round(p.stat().st_size / 1024 / 1024, 3) > min_mib]
    if not candidates:
        return None
    candidates.sort(key=lambda p: p.stat().st_size, reverse=True)
    return candidates[0]


def _mib(p: Path) -> float:
    return float(p.stat().st_size) / 1024.0 / 1024.0


def _print_candidates(base_dir: Path, label: str, patterns: list[str], min_mib: float) -> None:
    cands = _glob_candidates(base_dir, patterns)
    print(f"[DBG] {label}: searching in {base_dir} (non-recursive)")
    print(f"[DBG] {label}: patterns={patterns}, min_mib={min_mib}")
    if not cands:
        print(f"[DBG] {label}: no files matched patterns")
        return
    for p in sorted(cands):
        print(f"[DBG] {label}: candidate {p.name}  size={_mib(p):.3f} MiB  bytes={p.stat().st_size}")
    kept = [p for p in cands if _mib(p) > min_mib]
    if not kept:
        print(f"[DBG] {label}: all matched files are <= min_mib")
    else:
        best = max(kept, key=lambda x: x.stat().st_size)
        print(
            f"[DBG] {label}: best-by-size (after min_mib) = {best.name}  size={_mib(best):.3f} MiB  bytes={best.stat().st_size}"
        )


def _print_bag_info(bag_path: Path, focus_topics: list[str]) -> None:
    try:
        with rosbag.Bag(str(bag_path), "r") as bag:
            st = bag.get_start_time()
            et = bag.get_end_time()
            print(f"[DBG] bag: {bag_path.name}")
            print(f"[DBG]   start_time={st:.6f}  end_time={et:.6f}  duration={(et - st):.3f}s")
            info = bag.get_type_and_topic_info()
            topics = info.topics if hasattr(info, "topics") else {}
            print(f"[DBG]   num_topics={len(topics)}")
            for t in focus_topics:
                if t in topics:
                    print(f"[DBG]   topic[{t}] msg_count={topics[t].message_count}")
                else:
                    print(f"[DBG]   topic[{t}] NOT PRESENT in bag")
    except Exception as e:
        print(f"[ERR] failed to read bag info for {bag_path}: {e}")


def _summarize_intervals(times: list[float], intervals: list[tuple[float, float]]) -> None:
    if not times:
        print("[DBG] AP20 times: empty")
        return
    print(
        f"[DBG] AP20 times: count={len(times)}  t_min={times[0]:.6f}  t_max={times[-1]:.6f}  span={(times[-1]-times[0]):.3f}s"
    )
    if len(times) >= 6:
        print(f"[DBG] AP20 times: first3={[round(x,6) for x in times[:3]]}  last3={[round(x,6) for x in times[-3:]]}")
    if len(times) >= 2:
        gaps = [times[i] - times[i - 1] for i in range(1, len(times)) if times[i] > times[i - 1]]
        if gaps:
            m = statistics.median(gaps)
            T_gap = min(0.5, 3.0 * m)
            pad = 0.5 * m
            print(f"[DBG] AP20 gaps: n={len(gaps)}  median={m:.6f}s  T_gap={T_gap:.6f}s  pad={pad:.6f}s")
            print(f"[DBG] AP20 gaps: min={min(gaps):.6f}s  max={max(gaps):.6f}s")
        else:
            print("[DBG] AP20 gaps: none (all equal timestamps?)")
    if not intervals:
        print("[DBG] AP20 intervals: empty")
        return
    print(f"[DBG] AP20 intervals: count={len(intervals)}")
    for k, (a, b) in enumerate(intervals[:5], start=1):
        print(f"[DBG]   interval[{k}]: {a:.6f} -> {b:.6f}  len={(b-a):.3f}s")
    if len(intervals) > 5:
        a, b = intervals[-1]
        print(f"[DBG]   interval[last]: {a:.6f} -> {b:.6f}  len={(b-a):.3f}s")


# -------------------------
# Main automation
# -------------------------
def main(argv=None):
    parser = argparse.ArgumentParser(description="Distance-weighted AP20 availability")
    parser.add_argument("bag_dir", type=Path)
    args = parser.parse_args(argv)
    base_dir = args.bag_dir.resolve()
    print(f"[DBG] BASE_DIR={base_dir}")
    if not base_dir.exists() or not base_dir.is_dir():
        raise SystemExit(f"Not a directory: {base_dir}")

    # show what's in the directory (non-recursive)
    try:
        entries = list(base_dir.iterdir())
        files = [p.name for p in entries if p.is_file()]
        dirs = [p.name for p in entries if p.is_dir()]
        print(f"[DBG] base_dir entries: files={len(files)} dirs={len(dirs)}")
        if dirs:
            print(f"[DBG] base_dir subdirs (ignored): {sorted(dirs)[:20]}{' ...' if len(dirs) > 20 else ''}")
        if files:
            print(f"[DBG] base_dir files: {sorted(files)[:40]}{' ...' if len(files) > 40 else ''}")
    except Exception as e:
        print(f"[ERR] failed to list base_dir: {e}")

    _print_candidates(base_dir, "ODOM", ODOM_BAG_PATTERNS, MIN_BAG_MIB_ODO)
    _print_candidates(base_dir, "AP20", AP20_BAG_PATTERNS, MIN_BAG_MIB_AP20)

    # --- choose bags (largest by size) ---
    local_odom = select_best_local_bag(base_dir, ODOM_BAG_PATTERNS, MIN_BAG_MIB_ODO)
    if local_odom is None:
        raise SystemExit("No DLIO bag found (after min size filter); cannot compute distance/coverage")
    print(f"[DBG] selected ODOM bag: {local_odom}  size={_mib(local_odom):.3f} MiB  bytes={local_odom.stat().st_size}")

    local_ap20 = select_best_local_bag(base_dir, AP20_BAG_PATTERNS, MIN_BAG_MIB_AP20)
    if local_ap20 is None:
        print(f"[DBG] selected AP20 bag: NONE (after min size filter {MIN_AP20_BYTES} bytes) -> coverage will be null")
    else:
        print(
            f"[DBG] selected AP20 bag: {local_ap20}  size={_mib(local_ap20):.3f} MiB  bytes={local_ap20.stat().st_size}"
        )

    # --- inspect bags ---
    _print_bag_info(local_odom, focus_topics=[ODOM_TOPICS_CANDIDATES[0]])
    if local_ap20 is not None and local_ap20.exists():
        _print_bag_info(local_ap20, focus_topics=[AP20_TOPIC])

    # --- compute odom total + edges ---
    print(f"[DBG] computing odom edges/length on topic={ODOM_TOPICS_CANDIDATES[0]}")
    total_m, edges = compute_odom_edges_and_length(local_odom, ODOM_TOPICS_CANDIDATES[0])
    if total_m is None or not edges:
        raise SystemExit("No messages on DLIO odom topic (or no valid edges); cannot compute distance/coverage")

    print(f"[DBG] odom: total_m={total_m:.3f}  edges={len(edges)}")
    odom_t0 = edges[0][0]
    odom_t1 = edges[-1][1]
    print(f"[DBG] odom: t_min={odom_t0:.6f}  t_max={odom_t1:.6f}  span={(odom_t1 - odom_t0):.3f}s")

    # --- compute coverage ---
    coverage_pct = None

    if local_ap20 is None or (not Path(local_ap20).exists()):
        print("[ERR] AP20 bag missing -> coverage_pct=None")
        coverage_pct = None
    else:
        ap20_count = 0
        try:
            with rosbag.Bag(str(local_ap20), "r") as bag:
                ap20_count = bag.get_message_count(topic_filters=[AP20_TOPIC])
            print(f"[DBG] AP20 topic msg_count={ap20_count}")
        except Exception as e:
            print(f"[ERR] failed to count AP20 messages -> {e}")
            ap20_count = 0

        if ap20_count == 0:
            print(f"[ERR] AP20 topic {AP20_TOPIC} has zero messages -> coverage_pct=None")
            coverage_pct = None
        else:
            ap20_times = read_ap20_timestamps(local_ap20, AP20_TOPIC)
            print(f"[DBG] AP20 dedup timestamps count={len(ap20_times)}")
            gt_intervals = build_intervals_from_times(ap20_times)
            _summarize_intervals(ap20_times, gt_intervals)

            if not gt_intervals:
                print("[ERR] no AP20 intervals built -> coverage_pct=None")
                coverage_pct = None
            elif total_m <= 0.0:
                print("[ERR] total_m <= 0 -> coverage_pct=None")
                coverage_pct = None
            else:
                it0 = gt_intervals[0][0]
                it1 = gt_intervals[-1][1]
                print(f"[DBG] AP20 interval union time range: {it0:.6f} -> {it1:.6f}")
                if it1 <= odom_t0 or it0 >= odom_t1:
                    print("[ERR] AP20 intervals do not overlap odom time range -> coverage will be null/zero")

                d_cov = covered_distance_from_edges(edges, gt_intervals)
                print(f"[DBG] covered distance d_cov={d_cov:.3f} m (of total {total_m:.3f} m)")
                if d_cov <= 0.0:
                    print("[ERR] d_cov <= 0 -> coverage_pct=None")
                    coverage_pct = None
                else:
                    coverage_pct = round(100.0 * d_cov / total_m, 3)
                    print(f"[DBG] computed coverage_pct={coverage_pct:.3f}%")

    # Print in green
    GREEN = "\033[92m"
    RESET = "\033[0m"
    if coverage_pct is None:
        print(f"{GREEN}coverage_pct: null{RESET}")
    else:
        print(f"{GREEN}coverage_pct: {coverage_pct:.3f}%{RESET}")


if __name__ == "__main__":
    main()
