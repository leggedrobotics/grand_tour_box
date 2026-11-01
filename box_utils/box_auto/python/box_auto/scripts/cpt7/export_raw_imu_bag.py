#!/usr/bin/env python3
import argparse
import logging
import math
import os
import secrets
import sys
import threading
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional, Tuple, List

import genpy
import numpy as np
import pandas as pd
import rosbag
from box_auto.utils import MISSION_DATA
from boxi import ColorLogger

try:
    import tf.transformations as tft
except Exception:
    tft = None

"""
Exit Codes:
0: success
1: no RAWIMUSX.ASCII found
2: too many RAWIMUSX.ASCII found (single-file mode)
3: conversion/write failed
4: verify failed (missing bag/topic/deps)
"""

# `rosrun tf tf_echo cpt7_imu livox_imu` (cpt7 -> livox)
TF_T_CPT7_TO_LIVOX = np.array([0.30967, -0.02168, -0.15091], dtype=np.float64)
TF_Q_CPT7_TO_LIVOX_XYZW = np.array([0.00786, -0.00013, 0.00594, 0.99995], dtype=np.float64)

LIVOX_TOPIC_DEFAULT = "/gt_box/livox/imu_si_compliant"
CPT7_TOPIC_DEFAULT = "/gt_box/cpt7/offline_from_novatel_logs/imu"
DEFAULT_UTC_OFFSET = -18.0

PLOT_WINDOW_SECONDS = 5.0
PLOT_MIN_AVG_GYRO_RAD_S = 0.1
PLOT_MAX_TRIES = 300

FIG_DPI = 800  # metadata only for Pillow; OpenCV cannot embed DPI reliably


def configure_logger(debug: bool) -> logging.Logger:
    logger = ColorLogger.get_logger()
    logger.propagate = False

    # Deduplicate handlers (ColorLogger may add a handler per get_logger() call elsewhere)
    seen = set()
    for h in list(logger.handlers):
        key = (type(h), getattr(h, "stream", None))
        if key in seen:
            logger.removeHandler(h)
        else:
            seen.add(key)

    logger.setLevel(logging.DEBUG if debug else logging.INFO)
    return logger


def as_utc_str_from_ros_time(t: genpy.Time) -> str:
    return datetime.fromtimestamp(t.to_sec(), tz=timezone.utc).strftime("%Y-%m-%d-%H-%M-%S")


def parse_count_series(series: pd.Series) -> np.ndarray:
    if series.dtype == object:
        vals = series.astype(str).to_numpy()
        out = np.empty(vals.shape[0], dtype=np.int64)
        for i, s in enumerate(vals):
            s = s.strip()
            if "*" in s:
                s = s.split("*", 1)[0]
            out[i] = int(s)
        return out
    return series.to_numpy(dtype=np.int64)


def rotation_matrix_from_quat_xyzw(q_xyzw: np.ndarray) -> np.ndarray:
    if tft is None:
        raise RuntimeError("tf.transformations not available (ROS tf python bindings missing).")
    T = tft.quaternion_matrix(q_xyzw.tolist())
    return T[:3, :3]


def invert_transform(R: np.ndarray, t: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    R_inv = R.T
    t_inv = -R_inv @ t
    return R_inv, t_inv


def find_single_rawimusx_ascii(root: Path) -> Path:
    files = [p for p in root.rglob("*RAWIMUSX.ASCII")]
    if len(files) == 0:
        raise FileNotFoundError(f"No RAWIMUSX.ASCII found in: {root}")
    if len(files) > 1:
        raise RuntimeError(f"Too many ({len(files)}) RAWIMUSX.ASCII files found in: {root}")
    return files[0]


def find_livox_bag(root: Path, logger: logging.Logger) -> Path:
    matches = sorted(root.glob("*_nuc_livox.bag"))
    if not matches:
        matches = sorted(root.glob("*_nuc_livox*.bag"))
    if not matches:
        raise FileNotFoundError(f"No livox bag found in {root} matching '*_nuc_livox.bag'")
    if len(matches) > 1:
        matches.sort(key=lambda p: p.stat().st_mtime, reverse=True)
        logger.warning(f"[main] Multiple livox bags found; using newest: {matches[0].name}")
    return matches[0]


def derive_date_prefix_from_name(bag_path: Path) -> str:
    name = bag_path.name
    suffix = "_nuc_livox.bag"
    if name.endswith(suffix):
        return name[: -len(suffix)]
    # fallback: strip extension + optional '_nuc_livox' token
    stem = bag_path.stem  # no .bag
    if stem.endswith("_nuc_livox"):
        return stem[: -len("_nuc_livox")]
    return stem


def default_cpt7_out_bag_path(root: Path, date_prefix: str) -> Path:
    return root / f"{date_prefix}_cpt7_raw_imu.bag"


class RAWIMUDataParser:
    def __init__(self, output_imu_msg_name: str, utc_offset=DEFAULT_UTC_OFFSET, *, logger=None):
        self.logger = logger if logger is not None else logging.getLogger(__name__)

        # Preserve the established GPS - 18 s conversion by default.
        # If an independent clock reference confirms that a particular recording
        # needs an ADDITIONAL +37 s, explicitly pass --utc-offset 19 (or
        # utc_offset=19.0 here): -18 + 37 = +19. This shifts BOTH message-header
        # and bag-record timestamps by +37 s. The option replaces the GPS offset;
        # it is not an extra delta, so --utc-offset 37 would be incorrect.
        # Never apply this recording-specific adjustment automatically.
        self.expected_utc_offset = float(utc_offset)
        if not np.isfinite(self.expected_utc_offset):
            raise ValueError("UTC offset must be finite")
        self.expected_gps_offset = 0.0
        self.weeks_to_seconds = 604_800.0

        posix_start = datetime(1970, 1, 1, 0, 0, 0, tzinfo=timezone.utc)
        gps_start = datetime(1980, 1, 6, 0, 0, 0, tzinfo=timezone.utc)
        self.gps_epoch_start_in_POSIX_seconds = (gps_start - posix_start).total_seconds()

        self.x_accel = []
        self.minusy_accel = []
        self.z_accel = []
        self.x_gyro = []
        self.minusy_gyro = []
        self.z_gyro = []
        self.ros_times = []

        self.output_imu_msg_name = output_imu_msg_name
        self.reference_logfile_name = ""

    def process_imu_data(self, path: str) -> bool:
        self.logger.info(f"[convert] Reading RAWIMUSX ASCII: {path}")
        rawimu_df = self.fetch_dataframe_from_csv(path)
        if rawimu_df is None:
            self.logger.error(f"[convert] Could not process IMU file named: {path}")
            return False

        self.logger.info(f"[convert] Parsed CSV rows={len(rawimu_df)} cols={len(rawimu_df.columns)}")
        if not self.load_imu_times(rawimu_df):
            return False
        if not self.load_imu_data(rawimu_df):
            return False

        start_time = as_utc_str_from_ros_time(self.ros_times[0])
        end_time = as_utc_str_from_ros_time(self.ros_times[-1])
        self.logger.info(f"[convert] CPT7 RAWIMU time span (UTC): {start_time} -> {end_time}")

        self.reference_logfile_name = str(Path(path).stem)
        return True

    def load_imu_times(self, imu_df: pd.DataFrame) -> bool:
        if imu_df.empty:
            self.logger.error("RAWIMU file contains no samples")
            return False
        EXPECTED_IMU_FIELDNAME = "%RAWIMUSXA"
        if not (imu_df.iloc[:, 0] == EXPECTED_IMU_FIELDNAME).all():
            self.logger.error(
                f"Unexpected IMU message type.\nExpected: {EXPECTED_IMU_FIELDNAME}\nGot: {imu_df.iloc[0, 0]}"
            )
            return False

        WEEK_INDEX = 4
        SOW_INDEX = 5

        raw_week = np.asarray(imu_df.iloc[:, WEEK_INDEX], dtype=np.float64)
        sow = np.asarray(imu_df.iloc[:, SOW_INDEX], dtype=np.float64)
        if (
            not np.isfinite(raw_week).all()
            or not np.isfinite(sow).all()
            or np.any(raw_week < 0)
            or np.any(raw_week != np.floor(raw_week))
            or np.any(sow < 0)
            or np.any(sow >= self.weeks_to_seconds)
        ):
            raise ValueError("Invalid GPS week or seconds-of-week in RAWIMU input")
        week = raw_week.astype(np.int64)
        sow = sow - float(self.expected_gps_offset) + float(self.expected_utc_offset)

        secs_sow = np.floor(sow).astype(np.int64)
        nsecs = np.round((sow - secs_sow) * 1e9).astype(np.int64)

        # snap to 1000 ns and normalize
        nsecs = (np.round(nsecs / 1000.0).astype(np.int64)) * 1000
        carry = nsecs // 1_000_000_000
        secs_sow = secs_sow + carry
        nsecs = nsecs % 1_000_000_000

        seconds_since_gps_start = week * int(self.weeks_to_seconds) + secs_sow
        seconds_unix = seconds_since_gps_start + int(self.gps_epoch_start_in_POSIX_seconds)

        self.logger.info(f"[convert] RAWIMU starts at {datetime.fromtimestamp(int(seconds_unix[0]), tz=timezone.utc)}")
        self.ros_times = [genpy.Time(secs=int(s), nsecs=int(ns)) for s, ns in zip(seconds_unix, nsecs)]
        if any(b <= a for a, b in zip(self.ros_times, self.ros_times[1:])):
            raise ValueError("RAWIMU acquisition timestamps must be strictly increasing")
        return True

    def load_imu_data(self, imu_df: pd.DataFrame) -> bool:
        IMUTYPE_INDEX = 3
        if not (imu_df.iloc[:, IMUTYPE_INDEX] == 68).all():
            self.logger.error(f"IMU type {imu_df.iloc[0, IMUTYPE_INDEX]} doesn't match expected type 68")
            return False

        INDICES = {"Z Accel": -6, "-(Y Accel)": -5, "X Accel": -4, "Z Gyro": -3, "-Y Gyro": -2, "X Gyro": -1}
        CPT7RATE = 100.0
        GYROSCALEFACTOR = np.power(2.0, -33)
        ACCELSCALEFACTOR = np.power(2.0, -29)

        x_accel_cnt = parse_count_series(imu_df.iloc[:, INDICES["X Accel"]])
        my_accel_cnt = parse_count_series(imu_df.iloc[:, INDICES["-(Y Accel)"]])
        z_accel_cnt = parse_count_series(imu_df.iloc[:, INDICES["Z Accel"]])

        self.x_accel = x_accel_cnt * ACCELSCALEFACTOR * CPT7RATE
        self.minusy_accel = my_accel_cnt * ACCELSCALEFACTOR * CPT7RATE
        self.z_accel = z_accel_cnt * ACCELSCALEFACTOR * CPT7RATE

        x_gyro_cnt = parse_count_series(imu_df.iloc[:, INDICES["X Gyro"]])
        my_gyro_cnt = parse_count_series(imu_df.iloc[:, INDICES["-Y Gyro"]])
        z_gyro_cnt = parse_count_series(imu_df.iloc[:, INDICES["Z Gyro"]])

        self.x_gyro = x_gyro_cnt * GYROSCALEFACTOR * CPT7RATE
        self.minusy_gyro = my_gyro_cnt * GYROSCALEFACTOR * CPT7RATE
        self.z_gyro = z_gyro_cnt * GYROSCALEFACTOR * CPT7RATE

        self.logger.info(f"[convert] Loaded IMU samples: {len(self.x_accel)}")
        return True

    def write_to_rosbag(self, path: str) -> bool:
        processed_data = [
            self.ros_times,
            self.x_accel,
            self.minusy_accel,
            self.z_accel,
            self.x_gyro,
            self.minusy_gyro,
            self.z_gyro,
        ]
        lengths = [len(x) for x in processed_data]
        if len(set(lengths)) != 1 or not lengths[0]:
            self.logger.error(f"Not all timestamp and IMU data have the same lengths: {lengths}")
            return False

        from sensor_msgs.msg import Imu

        topic_name = self.output_imu_msg_name
        latest_message = Imu()
        latest_message.header.frame_id = "cpt7_imu"
        latest_message.header.seq = 0
        # RAWIMU supplies no orientation estimate. ROS uses -1 in this entry
        # to distinguish absent orientation from a zero-uncertainty estimate.
        latest_message.orientation_covariance[0] = -1.0
        if any(not np.isfinite(values).all() for values in processed_data[1:]):
            self.logger.error("RAWIMU contains nonfinite accelerometer or gyro values")
            return False

        try:
            from box_auto.utils.atomic_file import atomic_output

            with atomic_output(path) as temporary, rosbag.Bag(str(temporary), "w", compression="lz4") as bag:
                self.logger.info(f"[convert] Writing {lengths[0]} CPT7 samples to {path}")

                for stamp, x_a, minusy_a, z_a, x_g, minusy_g, z_g in zip(*processed_data):
                    latest_message.header.seq += 1
                    latest_message.header.stamp = stamp

                    latest_message.linear_acceleration.x = float(x_a)
                    latest_message.linear_acceleration.y = float(-minusy_a)
                    latest_message.linear_acceleration.z = float(z_a)

                    latest_message.angular_velocity.x = float(x_g)
                    latest_message.angular_velocity.y = float(-minusy_g)
                    latest_message.angular_velocity.z = float(z_g)

                    bag.write(topic=topic_name, msg=latest_message, t=stamp)

        except Exception as e:
            self.logger.error(f"[convert] Writing CPT7 IMU bag failed:\n{e}")
            return False

        self.logger.info(f"[convert] Wrote CPT7 IMU bag: {path}")
        return True

    def fetch_dataframe_from_csv(self, input_path: str) -> Optional[pd.DataFrame]:
        if not os.path.exists(input_path):
            self.logger.error(f"The path: {input_path} does not exist")
            return None
        try:
            # NovAtel ASCII may be headerless; pandas' default otherwise consumes
            # the first measurement as column names and silently loses a sample.
            with open(input_path, encoding="utf-8-sig") as stream:
                first_line = stream.readline().lstrip()
            return pd.read_csv(input_path, header=None if first_line.startswith("%RAWIMUSXA") else "infer")
        except Exception as e:
            self.logger.error(f"Failed to read CSV: {e}")
            return None


def is_imu_like(msg) -> bool:
    try:
        _ = msg.header.stamp
        _ = msg.angular_velocity.x
        _ = msg.angular_velocity.y
        _ = msg.angular_velocity.z
        _ = msg.linear_acceleration.x
        _ = msg.linear_acceleration.y
        _ = msg.linear_acceleration.z
        return True
    except Exception:
        return False


def stamp_for(msg, bag_time: genpy.Time) -> genpy.Time:
    if msg.header.stamp != genpy.Time(0):
        return msg.header.stamp
    return bag_time


def _slice_stats_sorted(
    t: np.ndarray, y: Optional[np.ndarray], start: float, end: float
) -> Tuple[int, Optional[float]]:
    """
    Fast slice stats for sorted timestamps using searchsorted.
    Returns (count, mean) where mean is None if y is None or count==0.
    """
    i0 = int(np.searchsorted(t, start, side="left"))
    i1 = int(np.searchsorted(t, end, side="right"))
    n = max(0, i1 - i0)
    if y is None or n == 0:
        return n, None
    return n, float(np.mean(y[i0:i1]))


def choose_random_overlap_window(
    livox_t: np.ndarray,
    cpt7_t: np.ndarray,
    window_s: float,
    seed: int,
    logger: logging.Logger,
    cpt7_gyr_amp: Optional[np.ndarray] = None,
    min_mean_gyro: Optional[float] = None,
    max_tries: int = PLOT_MAX_TRIES,
) -> Tuple[float, float, int]:
    tL0, tL1 = float(livox_t[0]), float(livox_t[-1])
    tC0, tC1 = float(cpt7_t[0]), float(cpt7_t[-1])

    ov0 = max(tL0, tC0)
    ov1 = min(tL1, tC1)
    if ov1 <= ov0:
        raise RuntimeError(f"No time overlap: livox [{tL0},{tL1}] vs cpt7 [{tC0},{tC1}]")

    ov_len = ov1 - ov0
    seed_used = int(seed) & 0xFFFFFFFF
    rng = np.random.default_rng(seed_used)

    if ov_len < window_s:
        logger.warning(f"[plot] Overlap only {ov_len:.3f}s (< {window_s}s). Plotting full overlap.")
        return ov0, ov1, seed_used

    # Original behavior if no constraint provided
    if cpt7_gyr_amp is None or min_mean_gyro is None:
        start = float(rng.uniform(ov0, ov1 - window_s))
        end = start + window_s
        logger.info(f"[plot] Overlap is {ov_len:.3f}s; selected random {window_s:.1f}s window.")
        return start, end, seed_used

    # Constrained selection: ensure CPT7 mean gyro amplitude in the window is >= min_mean_gyro
    best_start = None
    best_mean = -np.inf
    for _ in range(int(max_tries)):
        start = float(rng.uniform(ov0, ov1 - window_s))
        end = start + window_s

        nL, _ = _slice_stats_sorted(livox_t, None, start, end)
        nC, mC = _slice_stats_sorted(cpt7_t, cpt7_gyr_amp, start, end)
        if nL == 0 or nC == 0 or mC is None:
            continue

        if mC > best_mean:
            best_mean = mC
            best_start = start

        if mC >= float(min_mean_gyro):
            logger.info(
                f"[plot] Overlap is {ov_len:.3f}s; selected random {window_s:.1f}s window "
                f"meeting mean |w| >= {float(min_mean_gyro):.3f} rad/s (CPT7 mean={mC:.3f})."
            )
            return start, end, seed_used

    if best_start is not None and np.isfinite(best_mean):
        logger.warning(
            f"[plot] Could not find a {window_s:.1f}s window meeting mean |w| >= {float(min_mean_gyro):.3f} rad/s "
            f"after {int(max_tries)} tries. Using best window found (CPT7 mean={best_mean:.3f})."
        )
        return float(best_start), float(best_start + window_s), seed_used

    # Fallback (should be rare): just pick random
    logger.warning(
        f"[plot] Could not find any valid window with samples after {int(max_tries)} tries; using unconstrained random window."
    )
    start = float(rng.uniform(ov0, ov1 - window_s))
    return start, start + window_s, seed_used


# ---------------------------
# Plotting without matplotlib
# ---------------------------
def get_plot_backend(logger: logging.Logger):
    try:
        from PIL import Image, ImageDraw, ImageFont  # type: ignore

        logger.info("[plot] Using Pillow backend.")
        return "pil", (Image, ImageDraw, ImageFont)
    except Exception as e:
        logger.warning(f"[plot] Pillow import failed: {e}")

    try:
        import cv2  # type: ignore

        logger.info("[plot] Using OpenCV backend.")
        return "cv2", (cv2,)
    except Exception as e:
        logger.warning(f"[plot] OpenCV import failed: {e}")

    raise RuntimeError("No plotting backend available. Install Pillow (python3-pil) or OpenCV (python3-opencv).")


def nice_step(x: float) -> float:
    if not np.isfinite(x) or x <= 0.0:
        return 1.0
    exp = math.floor(math.log10(x))
    base = 10.0**exp
    f = x / base
    if f <= 1.0:
        nf = 1.0
    elif f <= 2.0:
        nf = 2.0
    elif f <= 5.0:
        nf = 5.0
    else:
        nf = 10.0
    return nf * base


def nice_ticks(vmin: float, vmax: float, nticks: int = 6) -> np.ndarray:
    if not (np.isfinite(vmin) and np.isfinite(vmax)):
        return np.array([-1.0, 0.0, 1.0], dtype=np.float64)
    if vmax < vmin:
        vmin, vmax = vmax, vmin
    if abs(vmax - vmin) < 1e-12:
        vmin -= 1.0
        vmax += 1.0
    span = vmax - vmin
    step = nice_step(span / max(1, nticks - 1))
    t0 = math.floor(vmin / step) * step
    t1 = math.ceil(vmax / step) * step
    n = int(round((t1 - t0) / step)) + 1
    ticks = t0 + step * np.arange(max(n, 2), dtype=np.float64)
    return ticks


def format_tick(v: float) -> str:
    av = abs(float(v))
    if (av > 0.0 and av < 1e-3) or av >= 1e4:
        return f"{v:.2e}"
    s = f"{v:.4f}".rstrip("0").rstrip(".")
    if s == "-0":
        s = "0"
    return s


def downsample_xy(x: np.ndarray, y: np.ndarray, max_points: int) -> Tuple[np.ndarray, np.ndarray]:
    if x.size <= max_points:
        return x, y
    idx = np.linspace(0, x.size - 1, num=max_points, dtype=np.int64)
    idx = np.unique(idx)
    return x[idx], y[idx]


def compute_y_bounds(series_list: List[np.ndarray]) -> Tuple[float, float]:
    y_min = float(np.min([np.min(s) for s in series_list]))
    y_max = float(np.max([np.max(s) for s in series_list]))
    if not np.isfinite(y_min) or not np.isfinite(y_max):
        y_min, y_max = -1.0, 1.0
    if abs(y_max - y_min) < 1e-12:
        y_min -= 1.0
        y_max += 1.0
    pad = 0.05 * (y_max - y_min)
    return y_min - pad, y_max + pad


def to_points(
    x: np.ndarray,
    y: np.ndarray,
    x0: float,
    x1: float,
    y0: float,
    y1: float,
    px0: int,
    py0: int,
    px1: int,
    py1: int,
) -> np.ndarray:
    xn = (x - x0) / (x1 - x0)
    yn = (y - y0) / (y1 - y0)
    xn = np.clip(xn, 0.0, 1.0)
    yn = np.clip(yn, 0.0, 1.0)

    px = (px0 + xn * (px1 - px0)).astype(np.int32)
    py = (py1 - yn * (py1 - py0)).astype(np.int32)
    return np.stack([px, py], axis=1)


def save_plot_png_amp(
    logger: logging.Logger,
    out_path: Path,
    title: str,
    x_livox: np.ndarray,
    y_livox_amp: np.ndarray,
    x_cpt7: np.ndarray,
    y_cpt7_amp: np.ndarray,
    y_label: str,
):
    backend, mods = get_plot_backend(logger)

    W, H = 3200, 1800
    # bigger margins to accommodate tick labels + axis labels
    margin_l, margin_r, margin_t, margin_b = 240, 80, 120, 220
    px0, py0 = margin_l, margin_t
    px1, py1 = W - margin_r, H - margin_b

    y0, y1 = compute_y_bounds([y_livox_amp, y_cpt7_amp])

    x0 = float(min(np.min(x_livox), np.min(x_cpt7)))
    x1 = float(max(np.max(x_livox), np.max(x_cpt7)))
    if x1 <= x0:
        x1 = x0 + 1.0

    # colors (BGR for cv2, RGB for PIL)
    livox_col = (20, 120, 220)
    cpt7_col = (140, 140, 140)

    # downsample to keep drawing cost bounded (one point per ~pixel column)
    max_pts = max(2000, (px1 - px0) * 2)
    x_livox_ds, y_livox_ds = downsample_xy(x_livox, y_livox_amp, max_pts)
    x_cpt7_ds, y_cpt7_ds = downsample_xy(x_cpt7, y_cpt7_amp, max_pts)

    xticks = nice_ticks(x0, x1, nticks=7)
    yticks = nice_ticks(y0, y1, nticks=7)

    if backend == "pil":
        Image, ImageDraw, ImageFont = mods  # type: ignore
        img = Image.new("RGB", (W, H), (255, 255, 255))
        draw = ImageDraw.Draw(img)

        # try a real TTF for readable labels; fallback to default bitmap font
        try:
            font = ImageFont.truetype("DejaVuSans.ttf", 28)
            font_small = ImageFont.truetype("DejaVuSans.ttf", 24)
            font_title = ImageFont.truetype("DejaVuSans.ttf", 34)
        except Exception:
            font = ImageFont.load_default()
            font_small = font
            font_title = font

        # plot area border
        draw.rectangle([px0, py0, px1, py1], outline=(0, 0, 0), width=3)

        # title
        draw.text((margin_l, 20), title, fill=(0, 0, 0), font=font_title)

        def text_size(text: str, fnt):
            try:
                b = draw.textbbox((0, 0), text, font=fnt)
                return (b[2] - b[0], b[3] - b[1])
            except Exception:
                return draw.textsize(text, font=fnt)

        # X ticks + grid
        for xv in xticks:
            if xv < x0 - 1e-12 or xv > x1 + 1e-12:
                continue
            xg = int(px0 + (float(xv) - x0) / (x1 - x0) * (px1 - px0))
            draw.line([(xg, py0), (xg, py1)], fill=(235, 235, 235), width=1)
            draw.line([(xg, py1), (xg, py1 + 12)], fill=(0, 0, 0), width=2)
            lab = format_tick(float(xv))
            tw, th = text_size(lab, font_small)
            draw.text((xg - tw // 2, py1 + 18), lab, fill=(0, 0, 0), font=font_small)

        # Y ticks + grid
        for yv in yticks:
            if yv < y0 - 1e-12 or yv > y1 + 1e-12:
                continue
            yg = int(py1 - (float(yv) - y0) / (y1 - y0) * (py1 - py0))
            draw.line([(px0, yg), (px1, yg)], fill=(235, 235, 235), width=1)
            draw.line([(px0 - 12, yg), (px0, yg)], fill=(0, 0, 0), width=2)
            lab = format_tick(float(yv))
            tw, th = text_size(lab, font_small)
            draw.text((px0 - 18 - tw, yg - th // 2), lab, fill=(0, 0, 0), font=font_small)

        # axis labels
        x_label = "t [s] (relative to segment start)"
        tw, th = text_size(x_label, font)
        draw.text(((px0 + px1) // 2 - tw // 2, H - margin_b + 120), x_label, fill=(0, 0, 0), font=font)

        # y label (rotated)
        try:
            y_img = Image.new("RGBA", (500, 70), (255, 255, 255, 0))
            y_draw = ImageDraw.Draw(y_img)
            y_draw.text((0, 0), y_label, fill=(0, 0, 0, 255), font=font)
            y_img = y_img.rotate(90, expand=True)
            img.paste(y_img, (40, (py0 + py1) // 2 - y_img.size[1] // 2), y_img)
        except Exception:
            draw.text((10, margin_t), y_label, fill=(0, 0, 0), font=font)

        # traces
        pts_l = to_points(x_livox_ds, y_livox_ds, x0, x1, y0, y1, px0, py0, px1, py1)
        draw.line([tuple(p) for p in pts_l], fill=livox_col, width=3)

        pts_c = to_points(x_cpt7_ds, y_cpt7_ds, x0, x1, y0, y1, px0, py0, px1, py1)
        draw.line([tuple(p) for p in pts_c], fill=cpt7_col, width=2)

        # legend (top-right inside plot area)
        legend_items = [("Livox (rotated to CPT7)", livox_col, 3), ("CPT7", cpt7_col, 2)]
        pad = 14
        line_len = 80
        lh = 44
        max_tw = 0
        for txt, _, _lw in legend_items:
            tw, th = text_size(txt, font_small)
            max_tw = max(max_tw, tw)
        box_w = pad * 3 + line_len + max_tw
        box_h = pad * 2 + lh * len(legend_items)

        bx1 = px1 - 14
        by0 = py0 + 14
        bx0 = bx1 - box_w
        by1 = by0 + box_h
        draw.rectangle([bx0, by0, bx1, by1], fill=(255, 255, 255), outline=(0, 0, 0), width=2)

        for i, (txt, col, lw) in enumerate(legend_items):
            y = by0 + pad + i * lh + lh // 2
            x_line0 = bx0 + pad
            x_line1 = x_line0 + line_len
            draw.line([(x_line0, y), (x_line1, y)], fill=col, width=lw)
            draw.text((x_line1 + pad, y - 16), txt, fill=(0, 0, 0), font=font_small)

        img.save(str(out_path), format="PNG", dpi=(FIG_DPI, FIG_DPI))
        return

    (cv2,) = mods  # type: ignore
    img = np.full((H, W, 3), 255, dtype=np.uint8)

    cv2.rectangle(img, (px0, py0), (px1, py1), (0, 0, 0), 3)

    # title
    cv2.putText(img, title, (margin_l, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 0, 0), 2, cv2.LINE_AA)

    # grid + ticks + labels
    tick_font = cv2.FONT_HERSHEY_SIMPLEX
    tick_scale = 0.75
    tick_th = 2
    tick_len = 12

    for xv in xticks:
        if xv < x0 - 1e-12 or xv > x1 + 1e-12:
            continue
        xg = int(px0 + (float(xv) - x0) / (x1 - x0) * (px1 - px0))
        cv2.line(img, (xg, py0), (xg, py1), (235, 235, 235), 1)
        cv2.line(img, (xg, py1), (xg, py1 + tick_len), (0, 0, 0), tick_th)
        lab = format_tick(float(xv))
        (tw, th), _ = cv2.getTextSize(lab, tick_font, tick_scale, tick_th)
        cv2.putText(
            img,
            lab,
            (xg - tw // 2, py1 + tick_len + th + 10),
            tick_font,
            tick_scale,
            (0, 0, 0),
            tick_th,
            cv2.LINE_AA,
        )

    for yv in yticks:
        if yv < y0 - 1e-12 or yv > y1 + 1e-12:
            continue
        yg = int(py1 - (float(yv) - y0) / (y1 - y0) * (py1 - py0))
        cv2.line(img, (px0, yg), (px1, yg), (235, 235, 235), 1)
        cv2.line(img, (px0 - tick_len, yg), (px0, yg), (0, 0, 0), tick_th)
        lab = format_tick(float(yv))
        (tw, th), _ = cv2.getTextSize(lab, tick_font, tick_scale, tick_th)
        cv2.putText(
            img,
            lab,
            (px0 - tick_len - 10 - tw, yg + th // 2),
            tick_font,
            tick_scale,
            (0, 0, 0),
            tick_th,
            cv2.LINE_AA,
        )

    # axis labels (cv2: keep y-label unrotated near top-left)
    cv2.putText(img, y_label, (20, margin_t + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2, cv2.LINE_AA)
    cv2.putText(
        img,
        "t [s] (relative to segment start)",
        (margin_l, H - 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        (0, 0, 0),
        2,
        cv2.LINE_AA,
    )

    pts_l = to_points(x_livox_ds, y_livox_ds, x0, x1, y0, y1, px0, py0, px1, py1)
    cv2.polylines(img, [pts_l.reshape((-1, 1, 2))], False, livox_col, 3, cv2.LINE_AA)

    pts_c = to_points(x_cpt7_ds, y_cpt7_ds, x0, x1, y0, y1, px0, py0, px1, py1)
    cv2.polylines(img, [pts_c.reshape((-1, 1, 2))], False, cpt7_col, 2, cv2.LINE_AA)

    # legend (top-right inside plot area)
    legend_items = [("Livox (rotated to CPT7)", livox_col, 3), ("CPT7", cpt7_col, 2)]
    pad = 14
    line_len = 80
    lh = 44
    max_tw = 0
    for txt, _, _lw in legend_items:
        (tw, _th), _ = cv2.getTextSize(txt, tick_font, 0.8, 2)
        max_tw = max(max_tw, tw)
    box_w = pad * 3 + line_len + max_tw
    box_h = pad * 2 + lh * len(legend_items)

    bx1 = px1 - 14
    by0 = py0 + 14
    bx0 = bx1 - box_w
    by1 = by0 + box_h
    cv2.rectangle(img, (bx0, by0), (bx1, by1), (255, 255, 255), thickness=-1)
    cv2.rectangle(img, (bx0, by0), (bx1, by1), (0, 0, 0), thickness=2)
    for i, (txt, col, lw) in enumerate(legend_items):
        y = by0 + pad + i * lh + lh // 2
        x0l = bx0 + pad
        x1l = x0l + line_len
        cv2.line(img, (x0l, y), (x1l, y), col, lw, cv2.LINE_AA)
        cv2.putText(img, txt, (x1l + pad, y + 10), tick_font, 0.8, (0, 0, 0), 2, cv2.LINE_AA)

    ok = cv2.imwrite(str(out_path), img)
    if not ok:
        raise RuntimeError(f"OpenCV failed to write image: {out_path}")


def verify_livox_against_cpt7(
    logger: logging.Logger,
    livox_bag_path: Path,
    livox_topic: str,
    cpt7_bag_path: Path,
    cpt7_topic: str,
    write_transformed_bag: Optional[Path],
    out_topic: str,
    out_frame_id: str,
    max_msgs: Optional[int],
) -> Tuple[Path, Path]:
    if tft is None:
        raise RuntimeError("tf.transformations not available; cannot compute rotation matrices.")

    logger.info(f"[verify] Livox bag:  {livox_bag_path}")
    logger.info(f"[verify] CPT7 bag:   {cpt7_bag_path}")
    logger.info(f"[verify] Livox topic: {livox_topic}")
    logger.info(f"[verify] CPT7 topic:  {cpt7_topic}")
    logger.info(f"[verify] max_msgs: {max_msgs if max_msgs is not None else 'ALL'}")

    R_livox_from_cpt7 = rotation_matrix_from_quat_xyzw(TF_Q_CPT7_TO_LIVOX_XYZW)
    R_cpt7_from_livox, _t_unused = invert_transform(R_livox_from_cpt7, TF_T_CPT7_TO_LIVOX)
    logger.info(f"[verify] Using rotation (livox->cpt7):\n{R_cpt7_from_livox}")
    logger.info("[verify] Translation ignored for IMU accel/gyro vectors.")

    bag_out = None
    if write_transformed_bag is not None:
        logger.info(f"[verify] Writing rotated Livox IMU to: {write_transformed_bag}")
        bag_out = rosbag.Bag(str(write_transformed_bag), "w", compression="lz4")

    livox_t, livox_acc, livox_gyr = [], [], []
    cpt7_t, cpt7_acc, cpt7_gyr = [], [], []

    try:
        logger.info("[verify] Reading Livox IMU messages...")
        with rosbag.Bag(str(livox_bag_path), "r") as bag:
            n = 0
            first_stamp = None
            last_stamp = None
            for _, msg, t in bag.read_messages(topics=[livox_topic]):
                if not is_imu_like(msg):
                    continue
                stamp = stamp_for(msg, t)
                if first_stamp is None:
                    first_stamp = stamp
                last_stamp = stamp

                av = np.array(
                    [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z], dtype=np.float64
                )
                la = np.array(
                    [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z], dtype=np.float64
                )

                av_cpt7 = R_cpt7_from_livox @ av
                la_cpt7 = R_cpt7_from_livox @ la

                livox_t.append(stamp.to_sec())
                livox_gyr.append(av_cpt7)
                livox_acc.append(la_cpt7)

                if bag_out is not None:
                    out_msg = deepcopy(msg)
                    out_msg.header.frame_id = out_frame_id
                    out_msg.angular_velocity.x = float(av_cpt7[0])
                    out_msg.angular_velocity.y = float(av_cpt7[1])
                    out_msg.angular_velocity.z = float(av_cpt7[2])
                    out_msg.linear_acceleration.x = float(la_cpt7[0])
                    out_msg.linear_acceleration.y = float(la_cpt7[1])
                    out_msg.linear_acceleration.z = float(la_cpt7[2])
                    bag_out.write(out_topic, out_msg, t=stamp)

                n += 1
                if max_msgs is not None and n >= max_msgs:
                    break

            logger.info(
                f"[verify] Livox stamp span (UTC): {as_utc_str_from_ros_time(first_stamp)} -> {as_utc_str_from_ros_time(last_stamp)}"
            )

        logger.info("[verify] Reading CPT7 IMU messages...")
        with rosbag.Bag(str(cpt7_bag_path), "r") as bag:
            n = 0
            first_stamp = None
            last_stamp = None
            for _, msg, t in bag.read_messages(topics=[cpt7_topic]):
                if not is_imu_like(msg):
                    continue
                stamp = stamp_for(msg, t)
                if first_stamp is None:
                    first_stamp = stamp
                last_stamp = stamp

                cpt7_t.append(stamp.to_sec())
                cpt7_gyr.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
                cpt7_acc.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

                n += 1
                if max_msgs is not None and n >= max_msgs:
                    break

            logger.info(
                f"[verify] CPT7 stamp span (UTC): {as_utc_str_from_ros_time(first_stamp)} -> {as_utc_str_from_ros_time(last_stamp)}"
            )

    finally:
        if bag_out is not None:
            bag_out.close()

    if len(livox_t) == 0:
        raise RuntimeError(f"No livox IMU-like messages read from {livox_bag_path} on {livox_topic}")
    if len(cpt7_t) == 0:
        raise RuntimeError(f"No cpt7 IMU-like messages read from {cpt7_bag_path} on {cpt7_topic}")

    livox_t = np.asarray(livox_t, dtype=np.float64)
    livox_acc = np.asarray(livox_acc, dtype=np.float64)
    livox_gyr = np.asarray(livox_gyr, dtype=np.float64)

    cpt7_t = np.asarray(cpt7_t, dtype=np.float64)
    cpt7_acc = np.asarray(cpt7_acc, dtype=np.float64)
    cpt7_gyr = np.asarray(cpt7_gyr, dtype=np.float64)

    # Precompute CPT7 gyro amplitude for constrained window selection
    cpt7_gyr_amp_full = np.linalg.norm(cpt7_gyr, axis=1)

    logger.info(
        f"[plot] Selecting a random {int(PLOT_WINDOW_SECONDS)}s segment inside the Livox/CPT7 time overlap "
        f"with mean |w| >= {PLOT_MIN_AVG_GYRO_RAD_S:.3f} rad/s (CPT7)..."
    )
    seed = secrets.randbits(32)
    seg_start, seg_end, seed_used = choose_random_overlap_window(
        livox_t=livox_t,
        cpt7_t=cpt7_t,
        window_s=PLOT_WINDOW_SECONDS,
        seed=seed,
        logger=logger,
        cpt7_gyr_amp=cpt7_gyr_amp_full,
        min_mean_gyro=PLOT_MIN_AVG_GYRO_RAD_S,
        max_tries=PLOT_MAX_TRIES,
    )
    logger.info(f"[plot] RNG seed: {seed_used}")
    logger.info(
        f"[plot] Segment (UTC): {datetime.fromtimestamp(seg_start, tz=timezone.utc)} -> {datetime.fromtimestamp(seg_end, tz=timezone.utc)}"
    )

    # Log achieved mean |w| for CPT7 on the chosen segment (if any samples)
    nC, mC = _slice_stats_sorted(cpt7_t, cpt7_gyr_amp_full, seg_start, seg_end)
    if nC > 0 and mC is not None:
        logger.info(f"[plot] Chosen segment CPT7 mean |w|: {mC:.3f} rad/s over {nC} samples")

    livox_mask = (livox_t >= seg_start) & (livox_t <= seg_end)
    cpt7_mask = (cpt7_t >= seg_start) & (cpt7_t <= seg_end)
    if not np.any(livox_mask):
        raise RuntimeError("[plot] No Livox samples in selected segment.")
    if not np.any(cpt7_mask):
        raise RuntimeError("[plot] No CPT7 samples in selected segment.")

    livox_t_seg = livox_t[livox_mask]
    livox_acc_seg = livox_acc[livox_mask]
    livox_gyr_seg = livox_gyr[livox_mask]

    cpt7_t_seg = cpt7_t[cpt7_mask]
    cpt7_acc_seg = cpt7_acc[cpt7_mask]
    cpt7_gyr_seg = cpt7_gyr[cpt7_mask]

    logger.info(f"[plot] Segment samples: Livox={livox_t_seg.size}, CPT7={cpt7_t_seg.size}")

    # relative time inside segment
    livox_tr = livox_t_seg - float(seg_start)
    cpt7_tr = cpt7_t_seg - float(seg_start)

    # amplitude (Euclidean norm)
    livox_acc_amp = np.linalg.norm(livox_acc_seg, axis=1)
    cpt7_acc_amp = np.linalg.norm(cpt7_acc_seg, axis=1)
    livox_gyr_amp = np.linalg.norm(livox_gyr_seg, axis=1)
    cpt7_gyr_amp = np.linalg.norm(cpt7_gyr_seg, axis=1)

    out_dir = livox_bag_path.parent
    base = livox_bag_path.stem
    accel_png = out_dir / f"{base}_livox_vs_cpt7_accel_amp_{int(PLOT_WINDOW_SECONDS)}s.png"
    gyro_png = out_dir / f"{base}_livox_vs_cpt7_gyro_amp_{int(PLOT_WINDOW_SECONDS)}s.png"

    testfile = out_dir / ".plot_write_test.tmp"
    try:
        testfile.write_text("ok")
        testfile.unlink()
    except Exception as e:
        raise RuntimeError(f"[plot] Cannot write to output directory {out_dir}: {e}") from e

    logger.info(f"[plot] Saving accel amplitude plot to: {accel_png}")
    save_plot_png_amp(
        logger=logger,
        out_path=accel_png,
        title=f"Accel amplitude in CPT7 frame ({int(PLOT_WINDOW_SECONDS)}s segment)",
        x_livox=livox_tr,
        y_livox_amp=livox_acc_amp,
        x_cpt7=cpt7_tr,
        y_cpt7_amp=cpt7_acc_amp,
        y_label="|a| [m/s^2]",
    )
    logger.info("[plot] Accel amplitude plot saved.")

    logger.info(f"[plot] Saving gyro amplitude plot to: {gyro_png}")
    save_plot_png_amp(
        logger=logger,
        out_path=gyro_png,
        title=f"Gyro amplitude in CPT7 frame ({int(PLOT_WINDOW_SECONDS)}s segment)",
        x_livox=livox_tr,
        y_livox_amp=livox_gyr_amp,
        x_cpt7=cpt7_tr,
        y_cpt7_amp=cpt7_gyr_amp,
        y_label="|w| [rad/s]",
    )
    logger.info("[plot] Gyro amplitude plot saved.")

    logger.info(f"[plot] Saved figures:\n  {accel_png}\n  {gyro_png}")
    return accel_png, gyro_png


def exit_guard(code: int) -> None:
    """
    Ensure the process actually terminates even if some library/debugger left things running.
    """
    # Try to let non-daemon threads finish briefly
    for _ in range(10):
        alive = [
            t for t in threading.enumerate() if t.is_alive() and (t is not threading.main_thread()) and (not t.daemon)
        ]
        if not alive:
            break
        for t in alive:
            t.join(timeout=0.2)

    # Best-effort flush/shutdown (don't risk blocking forever)
    try:
        logging.shutdown()
    except Exception:
        pass
    try:
        sys.stdout.flush()
        sys.stderr.flush()
    except Exception:
        pass

    alive_any = [t for t in threading.enumerate() if t.is_alive() and (t is not threading.main_thread())]
    if alive_any:
        try:
            sys.stderr.write(
                f"[exit] Forcing os._exit({code}); threads still alive: {[(t.name, t.daemon) for t in alive_any]}\n"
            )
            sys.stderr.flush()
        except Exception:
            pass

    os._exit(code)


def convert_all_rawimu(root, logger, utc_offset=DEFAULT_UTC_OFFSET):
    """Legacy --all writes each ASCII's bag alongside it for move_cpt7_files.py."""
    inputs = sorted(Path(root).rglob("*RAWIMUSX.ASCII"))
    if not inputs:
        raise FileNotFoundError(f"No RAWIMUSX.ASCII files in {root}")
    outputs = [source.with_name(source.name.split(".")[0] + ".bag") for source in inputs]
    if len(set(outputs)) != len(outputs):
        raise ValueError("Multiple RAWIMU inputs resolve to the same output bag")
    for source, output in zip(inputs, outputs):
        converter = RAWIMUDataParser(CPT7_TOPIC_DEFAULT, utc_offset=utc_offset, logger=logger)
        if not converter.process_imu_data(str(source)) or not converter.write_to_rosbag(str(output)):
            raise RuntimeError(f"RAWIMU conversion failed: {source}")
    return outputs


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate CPT7 RAW IMU bag from RAWIMUSX.ASCII, rotate Livox IMU into CPT7 frame, save 5s amplitude plots."
    )
    parser.add_argument("--skip_convert", action="store_true")
    parser.add_argument("--skip_verify", action="store_true")
    parser.add_argument(
        "--output_bag",
        type=Path,
        help="Explicit CPT7 output; with --skip_verify this removes the Livox input requirement",
    )
    parser.add_argument("--write_transformed_bag", default="")
    parser.add_argument("--max_msgs", type=int, default=0)
    parser.add_argument("--debug", action="store_true")
    parser.add_argument(
        "--all", action="store_true", help="Convert every ASCII beside its source (legacy batch workflow)"
    )
    parser.add_argument("--mission-data", type=Path, default=Path(MISSION_DATA))
    parser.add_argument(
        "--utc-offset",
        type=float,
        default=DEFAULT_UTC_OFFSET,
        help="Seconds added to GPS time (default: -18); explicitly selecting 19 adds 37 s relative to the default",
    )
    args = parser.parse_args()
    if not -86400 < args.utc_offset < 86400:
        parser.error("--utc-offset must be finite and within one day")
    if args.all and (args.skip_convert or args.output_bag or args.write_transformed_bag):
        parser.error("--all cannot be combined with single-output/verification options")

    logger = configure_logger(debug=args.debug)

    root = args.mission_data
    logger.info(f"[main] MISSION_DATA: {root}")
    logger.warning(
        "[convert] Using explicit time convention: GPS + %.6f s; verify against an independent UTC reference",
        args.utc_offset,
    )
    if args.all:
        convert_all_rawimu(root, logger, args.utc_offset)
        raise SystemExit(0)

    livox_bag = None
    if not args.skip_verify or args.output_bag is None:
        try:
            livox_bag = find_livox_bag(root, logger)
        except Exception as e:
            logger.error(
                f"[main] Failed to find Livox bag: {e}; use --skip_verify --output_bag PATH for standalone conversion"
            )
            raise SystemExit(4) from e
        logger.info(f"[main] Found Livox bag: {livox_bag.name}")

    cpt7_bag_out = (
        args.output_bag.expanduser()
        if args.output_bag is not None
        else default_cpt7_out_bag_path(root, derive_date_prefix_from_name(livox_bag))
    )
    cpt7_bag_out.parent.mkdir(parents=True, exist_ok=True)
    logger.info(f"[main] CPT7 output bag path: {cpt7_bag_out}")

    if not args.skip_convert:
        try:
            rawimu_ascii = find_single_rawimusx_ascii(root)
        except FileNotFoundError as e:
            logger.error(f"[main] {e}")
            raise SystemExit(1) from e
        except RuntimeError as e:
            logger.error(f"[main] {e}")
            raise SystemExit(2) from e

        logger.info(f"[main] Found RAWIMUSX ASCII: {rawimu_ascii}")

        imu_data_parser = RAWIMUDataParser(
            logger=logger, output_imu_msg_name=CPT7_TOPIC_DEFAULT, utc_offset=args.utc_offset
        )

        if not imu_data_parser.process_imu_data(str(rawimu_ascii)):
            logger.error("[main] Conversion failed while processing RAWIMUSX ASCII.")
            raise SystemExit(3)

        if not imu_data_parser.write_to_rosbag(str(cpt7_bag_out)):
            logger.error("[main] Conversion failed while writing CPT7 bag.")
            raise SystemExit(3)

    if args.skip_verify:
        logger.info("[main] skip_verify set; exiting after conversion.")
        raise SystemExit(0)

    if not cpt7_bag_out.exists():
        logger.error(f"[main] Expected CPT7 bag does not exist: {cpt7_bag_out} (did you --skip_convert?)")
        raise SystemExit(4)

    max_msgs = None if args.max_msgs <= 0 else int(args.max_msgs)
    write_bag = Path(args.write_transformed_bag) if args.write_transformed_bag.strip() else None

    try:
        verify_livox_against_cpt7(
            logger=logger,
            livox_bag_path=livox_bag,
            livox_topic=LIVOX_TOPIC_DEFAULT,
            cpt7_bag_path=cpt7_bag_out,
            cpt7_topic=CPT7_TOPIC_DEFAULT,
            write_transformed_bag=write_bag,
            out_topic="/gt_box/livox/imu_si_compliant_in_cpt7",
            out_frame_id="cpt7_imu",
            max_msgs=max_msgs,
        )
    except Exception as e:
        logger.error(f"[main] Verification failed: {e}")
        raise SystemExit(4) from e

    logger.info("[main] Done.")
    raise SystemExit(0)


if __name__ == "__main__":
    code = 0
    try:
        main()
    except SystemExit as e:
        code = int(e.code) if e.code is not None else 0
    exit_guard(code)
