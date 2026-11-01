#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional
import os
import shutil
import textwrap

# Hard-coded (non-recursive) directory containing ROS1 Noetic *.bag files

# Set to 0 to auto-pick (CPU-1). Set to 1 to disable multiprocessing.
WORKERS = 0


# ---------- Color helpers (ANSI) ----------
class Ansi:
    RESET = "\x1b[0m"
    BOLD = "\x1b[1m"
    DIM = "\x1b[2m"

    RED = "\x1b[31m"
    GREEN = "\x1b[32m"
    YELLOW = "\x1b[33m"
    CYAN = "\x1b[36m"
    GRAY = "\x1b[90m"


def _use_color() -> bool:
    return os.getenv("NO_COLOR") is None and hasattr(os.sys.stdout, "isatty") and os.sys.stdout.isatty()


def _c(s: str, code: str, enabled: bool) -> str:
    if not enabled:
        return s
    return f"{code}{s}{Ansi.RESET}"


# ---------- Bag reading ----------
@dataclass(frozen=True)
class BagResult:
    bag_name: str
    topics: tuple[str, ...]
    error: Optional[str] = None


def _topics_in_bag(bag_path_str: str) -> BagResult:
    # Import inside worker for multiprocessing compatibility
    try:
        import rosbag  # type: ignore

        p = Path(bag_path_str)
        with rosbag.Bag(str(p), mode="r") as bag:
            info = bag.get_type_and_topic_info()  # fast: metadata only
            topics = tuple(sorted(info.topics.keys()))
        return BagResult(bag_name=p.name, topics=topics, error=None)
    except Exception as e:
        p = Path(bag_path_str)
        return BagResult(bag_name=p.name, topics=tuple(), error=repr(e))


# ---------- Table printing ----------
def _format_table(results: list[BagResult]) -> str:
    color = _use_color()
    term_width = shutil.get_terminal_size(fallback=(120, 24)).columns

    bag_header = "BAG"
    topics_header = "TOPICS"

    bag_col_width = max(len(bag_header), *(len(r.bag_name) for r in results))
    bag_col_width = min(bag_col_width, 60)  # keep sane if filenames are huge

    # 3 separators: "| ", " | ", " |"
    topics_col_width = max(20, term_width - (bag_col_width + 6))

    def hline() -> str:
        return "+" + "-" * (bag_col_width + 2) + "+" + "-" * (topics_col_width + 2) + "+"

    out: list[str] = []
    out.append(hline())
    out.append(
        f"| {_c(bag_header.ljust(bag_col_width), Ansi.BOLD + Ansi.YELLOW, color)} "
        f"| {_c(topics_header.ljust(topics_col_width), Ansi.BOLD + Ansi.YELLOW, color)} |"
    )
    out.append(hline())

    for r in sorted(results, key=lambda x: x.bag_name.lower()):
        bag_cell = r.bag_name[:bag_col_width].ljust(bag_col_width)

        if r.error:
            msg = f"ERROR: {r.error}"
            wrapped = textwrap.wrap(msg, width=topics_col_width, break_long_words=False, break_on_hyphens=False) or [""]
            for i, line in enumerate(wrapped):
                left = _c(bag_cell, Ansi.CYAN, color) if i == 0 else " " * bag_col_width
                right = _c(line.ljust(topics_col_width), Ansi.RED, color)
                out.append(f"| {left} | {right} |")
            out.append(hline())
            continue

        topics_str = ", ".join(r.topics) if r.topics else "(no topics)"
        wrapped = textwrap.wrap(topics_str, width=topics_col_width, break_long_words=False, break_on_hyphens=False) or [
            ""
        ]

        for i, line in enumerate(wrapped):
            left = _c(bag_cell, Ansi.CYAN, color) if i == 0 else " " * bag_col_width
            right = _c(line.ljust(topics_col_width), Ansi.GREEN, color)
            out.append(f"| {left} | {right} |")

        out.append(hline())

    return "\n".join(out)


def main(argv=None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description="Inspect ROS bag topic metadata")
    parser.add_argument("bag_dir", type=Path)
    parser.add_argument("--workers", type=int, default=1)
    args = parser.parse_args(argv)
    BAG_DIR = args.bag_dir
    if args.workers < 0:
        parser.error("workers must be nonnegative")
    if not BAG_DIR.is_dir():
        raise SystemExit(f"Not a directory: {BAG_DIR}")

    bag_paths = sorted(BAG_DIR.glob("*.bag"))
    if not bag_paths:
        print(f"No .bag files found in: {BAG_DIR}")
        return 0

    n_workers = min(args.workers, len(bag_paths))
    if n_workers == 0:
        n_workers = min(len(bag_paths), max(1, (os.cpu_count() or 2) - 1))

    results: list[BagResult] = []

    if n_workers == 1:
        for p in bag_paths:
            results.append(_topics_in_bag(str(p)))
    else:
        import multiprocessing as mp

        ctx = mp.get_context("spawn")
        with ctx.Pool(processes=n_workers) as pool:
            for r in pool.imap_unordered(_topics_in_bag, (str(p) for p in bag_paths), chunksize=8):
                results.append(r)

    print(_format_table(results))

    # Non-zero if any bag failed
    return 2 if any(r.error for r in results) else 0


if __name__ == "__main__":
    raise SystemExit(main())
