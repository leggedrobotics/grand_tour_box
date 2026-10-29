"""Distance-weighted AP20 availability; coverage assumes uniform speed per odometry edge.

The existing gap policy is min(0.5 s, 3 * median sampling interval), with
half-interval padding. This measures availability, not ground-truth accuracy.
"""

from __future__ import annotations
import math
import statistics
from pathlib import Path
import rosbag


def _stamp_to_float(stamp) -> float:
    """Convert ROS time to float seconds."""
    return float(stamp.secs) + float(stamp.nsecs) * 1e-9


def compute_odom_edges_and_length(bag_path: Path, topic: str) -> tuple[float | None, list[tuple[float, float, float]]]:
    """
    Return (total path length [m], edges) from Odometry on `topic`;
    - total None if no messages.
    - edges = list of (t_prev, t_curr, step_len_m)
    """
    edges: list[tuple[float, float, float]] = []

    with rosbag.Bag(str(bag_path), "r") as bag:
        if bag.get_message_count(topic_filters=[topic]) == 0:
            return None, edges

        total_m = 0.0
        prev_p = None  # (x, y, z)
        prev_t = None  # float seconds

        for _, msg, t in bag.read_messages(topics=[topic]):
            # Prefer header.stamp if available; otherwise fall back to bag time `t`
            ts = _stamp_to_float(msg.header.stamp) if hasattr(msg, "header") else float(t.to_sec())
            p = msg.pose.pose.position
            curr_p = (float(p.x), float(p.y), float(p.z))

            if not math.isfinite(ts) or ts <= 0 or not all(math.isfinite(v) for v in curr_p):
                raise ValueError("Odometry requires finite positions and positive header timestamps")
            if prev_t is not None and ts <= prev_t:
                raise ValueError("Odometry timestamps must be strictly increasing")
            if prev_p is not None and prev_t is not None:
                dx = curr_p[0] - prev_p[0]
                dy = curr_p[1] - prev_p[1]
                dz = curr_p[2] - prev_p[2]
                step = math.sqrt(dx * dx + dy * dy + dz * dz)

                # guard corrupt jumps
                if step >= 1000.0:
                    raise ValueError("Odometry jump >= 1000 m; cannot report a trustworthy distance")
                if step < 1000.0:
                    dt = ts - prev_t
                    if dt > 0.0:
                        total_m += step
                        edges.append((prev_t, ts, step))
            prev_p = curr_p
            prev_t = ts

    return total_m, edges


def read_ap20_timestamps(bag_path: Path, topic: str) -> list[float]:
    """Extract PointStamped header timestamps (float seconds) from AP20 bag."""
    ts: list[float] = []
    with rosbag.Bag(str(bag_path), "r") as bag:
        if bag.get_message_count(topic_filters=[topic]) == 0:
            return ts
        for _, msg, t in bag.read_messages(topics=[topic]):
            if hasattr(msg, "header"):
                ts.append(_stamp_to_float(msg.header.stamp))
            else:
                ts.append(float(t.to_sec()))
    if any(not math.isfinite(t) or t <= 0 for t in ts):
        raise ValueError("AP20 requires finite positive acquisition timestamps")
    # enforce strictly increasing by sorting and dropping duplicates
    ts.sort()
    dedup: list[float] = []
    last = None
    for v in ts:
        if (last is None) or (v > last):
            dedup.append(v)
            last = v
    return dedup


def build_intervals_from_times(times: list[float]) -> list[tuple[float, float]]:
    """
    Build continuous-availability intervals from sample times using an adaptive gap threshold.
    Returns sorted, merged, non-overlapping intervals [(start, end), ...].
    """
    if any(not math.isfinite(t) for t in times) or any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError("Availability times must be finite and strictly increasing")
    if len(times) < 2:
        return []

    gaps = [times[i] - times[i - 1] for i in range(1, len(times)) if times[i] > times[i - 1]]
    if not gaps:
        return []

    m = statistics.median(gaps)
    # adaptive threshold: up to 0.5s, otherwise 3x median gap
    T_gap = min(0.5, 3.0 * m)
    pad = 0.5 * m  # conservative padding to counter jitter

    intervals: list[tuple[float, float]] = []
    start = times[0]
    prev = times[0]
    for t in times[1:]:
        if (t - prev) > T_gap:
            intervals.append((start - pad, prev + pad))
            start = t
        prev = t
    intervals.append((start - pad, prev + pad))

    # clip invalid and merge overlaps (due to padding)
    cleaned = [(a, b) for (a, b) in intervals if b > a]
    cleaned.sort()
    merged: list[list[float]] = []
    for a, b in cleaned:
        if not merged or a > merged[-1][1]:
            merged.append([a, b])
        else:
            merged[-1][1] = max(merged[-1][1], b)
    return [(a, b) for a, b in merged]


def covered_distance_from_edges(edges: list[tuple[float, float, float]], intervals: list[tuple[float, float]]) -> float:
    """
    Compute distance covered within the union of availability intervals by
    weighting each odom edge length by the fractional temporal overlap.
    """
    if not edges or not intervals:
        return 0.0

    d_cov = 0.0
    i = 0  # interval pointer
    nI = len(intervals)

    for t0, t1, step in edges:
        if t1 <= t0:
            continue
        dt = t1 - t0
        # advance intervals that end before the edge
        while i < nI and intervals[i][1] <= t0:
            i += 1
        j = i
        overlap_time = 0.0
        # accumulate overlaps with subsequent intervals until beyond t1
        while j < nI and intervals[j][0] < t1:
            a, b = intervals[j]
            s = max(t0, a)
            e = min(t1, b)
            if e > s:
                overlap_time += e - s
            if b >= t1:
                break
            j += 1
        if overlap_time > 0.0:
            d_cov += step * (overlap_time / dt)

    return d_cov


def input_provenance(path, topic):
    """Identify the exact source bag even after a temporary download is removed."""
    import hashlib
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return {"file": Path(path).name, "topic": topic, "sha256": digest.hexdigest(),
            "clock": "header.stamp", "jump_limit_m": 1000.0}


def append_summary(path, row):
    import yaml
    from box_auto.utils.atomic_file import atomic_output
    path = Path(path)
    existing = []
    if path.exists() and path.stat().st_size:
        existing = yaml.safe_load(path.read_text())
        if not isinstance(existing, list):
            raise ValueError("Existing statistics output must be a YAML list")
    with atomic_output(path) as temporary:
        temporary.write_text(yaml.safe_dump(existing + [row], sort_keys=False))
