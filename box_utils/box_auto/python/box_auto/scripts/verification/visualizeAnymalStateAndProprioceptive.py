#!/usr/bin/env python3
# Read ROS1 .bag and plot (within a requested time window relative to bag start time):
#  - anymal_msgs/AnymalState joint estimates + contacts
#  - series_elastic_actuator_msgs/SeActuatorReadings joint actuator readings (+ commanded)
#  - sensor_msgs/Imu (if present)  [ALL IMU TOPICS OVERLAID + ALIASES]  (X-axis components only)
#  - geometry_msgs/PointStamped (all topics of that type)
#  - geometry_msgs/TwistStamped command vs nav_msgs/Odometry twist (cmd vs state_estimator/odometry)
#
# Saves figures to OUTPUT_DIR as PDF at dpi=800.
#
# Dependencies: rosbags, numpy, matplotlib

from __future__ import annotations

import math
import re
import sys
import time
from pathlib import Path
from collections import defaultdict
from typing import Iterable

import numpy as np

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from rosbags.highlevel import AnyReader


# =========================
# HARD-CODED PARAMETERS
# =========================
BAG_PATH = None  # Set by the explicit CLI.
OUTPUT_DIR = None  # Set by the explicit CLI.
DPI = 800

# Time window relative to bag start time (seconds).
START_TIME_S = 15.0
END_TIME_S = 20.0  # set to None to plot until end of bag

# Optional plot decimation (points per series). Set to None to disable.
MAX_POINTS_PER_SERIES = 20000

# Optional extra per-joint plots (many files)
PLOT_PER_JOINT_FIGS = False

# Restrict joint plots to a single leg (3 joints). Options: "LF", "RF", "LH", "RH", or None for all legs.
ONLY_LEG = "LF"

# Preferred topics for twist comparison (used if present; otherwise falls back to "best" topic found)
TWIST_CMD_TOPIC_HINT = "/anymal/twist_command"
ODOM_TOPIC_HINT = "/anymal/state_estimator/odometry"

# IMU aliases for legend (topic -> short name). Suffix matches are supported.
IMU_TOPIC_ALIASES: dict[str, str] = {
    "/anymal/imu": "ANYmal",  # n=2000
    "/boxi/adis/imu": "Adis",  # n=1005
    "/boxi/alphasense/imu": "CoreResearch",  # n=998
    "/boxi/ap20/imu": "AP20",  # n=1000
    "/boxi/cpt7/imu": "HG4930",  # n=500
    "/boxi/livox/imu": "Livox",  # n=1000
    "/boxi/stim320/imu": "Stim320",  # n=2500
}

# IMU accel plotted in g's (divide by standard gravity)
G0_MPS2 = 9.8065

# Contact visualization spacing (avoid overlaps)
CONTACT_ROW_SPACING = 2.0  # vertical spacing between contacts in the state subplot
CONTACT_STATE_AMPL = 0.8  # state amplitude within a row (must be < CONTACT_ROW_SPACING)

# -------------------------
# FIGURE SIZING (ADJUSTABLE)
# -------------------------
# Global width used by:
#   - joints_* figures
#   - twist_command_vs_odometry figure
#   - contacts + IMU
FIG_WIDTH_IN = 4.5

# IMU figure height.
IMU_FIG_HEIGHT_IN = 2.5

# Joint grid figure height model:
#   height = max(min_height, base + per_row * nrows)
# Reduce these to compress joints_* figures.
JOINTS_FIG_MIN_HEIGHT_IN = 1.0
JOINTS_FIG_BASE_HEIGHT_IN = 0.8
JOINTS_FIG_HEIGHT_PER_ROW_IN = 0.55

# Twist command vs odometry figure height (2x3 grid).
# Reduce this to compress twist_command_vs_odometry.
TWIST_FIG_HEIGHT_IN = 2.5


# Contacts figure height model:
#   height = max(min_height, base + per_contact * n_contacts)
CONTACT_FIG_MIN_HEIGHT_IN = 2.5
CONTACT_FIG_BASE_HEIGHT_IN = 1.5
CONTACT_FIG_HEIGHT_PER_CONTACT_IN = 0.18

# Contacts subplot height ratios (state vs force).
CONTACT_HEIGHT_RATIOS = (1.1, 0.9)

# Legend formatting to avoid overlap/clipping in compressed figures.
LEGEND_MAX_NCOL = 6
LEGEND_FONTSIZE = 6

# Contacts ytick label font size (helpful when many contacts).
CONTACT_YTICK_FONTSIZE = 6

# Text backgrounds (ensure readability over plotted lines)
TEXT_BBOX = dict(facecolor="white", edgecolor="none", alpha=1.0, boxstyle="square,pad=0.15")
TEXT_ZORDER = 20
LEGEND_FRAMEALPHA = 1.0
# -------------------------

# Progress bar update throttling (keeps overhead low)
PROGRESS_UPDATE_PERIOD_S = 0.10  # wall-clock seconds between prints
PROGRESS_UPDATE_EVERY_N_MSG = 200  # also allow update every N msgs

# Canonical ANYmal joint order (fallback + nicer ordering)
ANYMAL_JOINT_NAMES = [
    "LF_HAA",
    "LF_HFE",
    "LF_KFE",
    "RF_HAA",
    "RF_HFE",
    "RF_KFE",
    "LH_HAA",
    "LH_HFE",
    "LH_KFE",
    "RH_HAA",
    "RH_HFE",
    "RH_KFE",
]

# Message type names (ROS1)
T_ANYMAL_STATE = "anymal_msgs/AnymalState"
T_SEA_READINGS = "series_elastic_actuator_msgs/SeActuatorReadings"
T_IMU = "sensor_msgs/Imu"
T_POINTSTAMPED = "geometry_msgs/PointStamped"
T_TWISTSTAMPED = "geometry_msgs/TwistStamped"
T_ODOMETRY = "nav_msgs/Odometry"


# =========================
# TYPE NORMALIZATION
# =========================
def _norm_type(t: str | None) -> str:
    s = (t or "").strip()
    s = s.replace("/msg/", "/").replace("::", "/")
    return s


def _is_type(actual: str, want: str) -> bool:
    return _norm_type(actual) == _norm_type(want)


# =========================
# SEQ HELPERS (avoid numpy truthiness)
# =========================
def _as_list(x) -> list:
    if x is None:
        return []
    if isinstance(x, np.ndarray):
        return x.tolist()
    return list(x)


def _as_np_float(x) -> np.ndarray:
    if x is None:
        return np.empty((0,), dtype=np.float64)
    return np.asarray(x, dtype=np.float64)


# =========================
# TIME HELPERS
# =========================
def _s_to_ns(s: float) -> int:
    return int(round(float(s) * 1e9))


def _ns_to_rel_s(t_ns: np.ndarray, t0_ns: int) -> np.ndarray:
    return (t_ns.astype(np.float64) - float(t0_ns)) / 1e9


def _apply_xlim(ax):
    if END_TIME_S is None:
        ax.set_xlim(left=float(START_TIME_S))
    else:
        ax.set_xlim(float(START_TIME_S), float(END_TIME_S))


# =========================
# STRING / NAMING HELPERS
# =========================
_joint_token_re = re.compile(r"(LF|RF|LH|RH)_(HAA|HFE|KFE)")


def _sanitize(s: str) -> str:
    s = (s or "").strip()
    s = s.replace(" ", "_").replace("/", "_")
    s = re.sub(r"[^A-Za-z0-9_.-]", "_", s)
    s = re.sub(r"_+", "_", s).strip("_")
    return s or "unknown"


def _normalize_joint_name(name: str | None, idx_fallback: int | None) -> str:
    raw = (name or "").strip()
    m = _joint_token_re.search(raw)
    if m:
        return f"{m.group(1)}_{m.group(2)}"
    if idx_fallback is not None and 0 <= idx_fallback < len(ANYMAL_JOINT_NAMES):
        return ANYMAL_JOINT_NAMES[idx_fallback]
    if raw:
        return _sanitize(raw)
    return f"joint_{idx_fallback if idx_fallback is not None else 0}"


def _filter_joints_by_leg(joints: list[str], only_leg: str | None) -> list[str]:
    if only_leg is None:
        return joints
    only_leg = str(only_leg).strip().upper()
    if only_leg not in ("LF", "RF", "LH", "RH"):
        return joints
    prefix = only_leg + "_"
    return [j for j in joints if j.startswith(prefix)]


def _basename_topic(topic: str) -> str:
    t = (topic or "").strip()
    if not t:
        return "unknown"
    if "/" in t:
        b = t.rsplit("/", 1)[-1].strip()
        return b if b else t
    return t


def _imu_device_after_boxi(topic: str) -> str | None:
    toks = [x for x in (topic or "").split("/") if x]
    if not toks:
        return None

    if "boxi" in toks:
        i = toks.index("boxi")
        if i + 2 < len(toks) and toks[i + 2] == "imu":
            return toks[i + 1]
        if toks[-1] == "imu" and i + 1 < len(toks):
            return toks[i + 1]

    if len(toks) >= 2 and toks[-1] == "imu":
        return toks[-2]

    return None


def _pretty_imu_name_from_topic(topic: str) -> str:
    dev = _imu_device_after_boxi(topic)
    if dev:
        return dev
    b = _basename_topic(topic)
    b2 = re.sub(r"[^A-Za-z0-9]+", "_", b).strip("_")
    return b2 if b2 else b


def _imu_alias(topic: str) -> str:
    if topic in IMU_TOPIC_ALIASES:
        return IMU_TOPIC_ALIASES[topic]
    for k, v in IMU_TOPIC_ALIASES.items():
        if k and (topic == k or topic.endswith(k)):
            return v
    return _pretty_imu_name_from_topic(topic)


def _imu_draw_priority(topic: str) -> int:
    """
    Lower draws first (behind). Higher draws last (in front).
    Enforce: Livox behind everything, CPT7 in front of everything.
    """
    t = (topic or "").lower()
    if "livox" in t:
        return 0
    if "cpt7" in t:
        return 2
    return 1


def _imu_zorder(topic: str) -> float:
    p = _imu_draw_priority(topic)
    if p == 0:
        return 1.0  # very back
    if p == 2:
        return 10.0  # very front
    return 5.0  # middle


# =========================
# DATA STRUCTURES
# =========================
est_data = defaultdict(lambda: defaultdict(lambda: defaultdict(list)))
sea_data = defaultdict(lambda: defaultdict(lambda: defaultdict(list)))
contacts_data = defaultdict(lambda: defaultdict(lambda: defaultdict(list)))
imu_data = defaultdict(lambda: defaultdict(list))
point_data = defaultdict(lambda: defaultdict(list))

twist_cmd_data = defaultdict(lambda: defaultdict(list))  # topic -> fields
odom_twist_data = defaultdict(lambda: defaultdict(list))  # topic -> fields

global_min_ns = None
global_max_ns = None


def _track_time(ns: int):
    global global_min_ns, global_max_ns
    if global_min_ns is None or ns < global_min_ns:
        global_min_ns = int(ns)
    if global_max_ns is None or ns > global_max_ns:
        global_max_ns = int(ns)


def _safe_float(x, default=np.nan) -> float:
    try:
        return float(x)
    except Exception:
        return float(default)


def _force_norm_from_wrench(wrench) -> float:
    try:
        f = wrench.force
        return float(math.sqrt(float(f.x) ** 2 + float(f.y) ** 2 + float(f.z) ** 2))
    except Exception:
        return float("nan")


def _get_attr_any(obj, names: Iterable[str], default=np.nan) -> float:
    if obj is None:
        return float(default)
    for n in names:
        try:
            if hasattr(obj, n):
                v = getattr(obj, n)
                if v is None:
                    continue
                return float(v)
        except Exception:
            continue
    return float(default)


# =========================
# PROGRESS BAR
# =========================
class _ProgressBar:
    def __init__(
        self,
        *,
        label: str,
        bag_start_ns: int,
        total_end_ns: int | None,
        width: int = 28,
        update_period_s: float = 0.10,
        update_every_n: int = 200,
        stream=None,
    ):
        self.label = label
        self.bag_start_ns = int(bag_start_ns)
        self.total_end_ns = int(total_end_ns) if total_end_ns is not None else None
        self.width = int(width)
        self.update_period_s = float(update_period_s)
        self.update_every_n = int(update_every_n)
        self.stream = stream if stream is not None else sys.stderr

        self._t0 = time.monotonic()
        self._last_print = 0.0
        self._last_ratio = -1.0

    def _format_bar(self, ratio: float) -> str:
        r = max(0.0, min(1.0, float(ratio)))
        done = int(round(r * self.width))
        done = max(0, min(self.width, done))
        return "[" + ("#" * done) + ("-" * (self.width - done)) + "]"

    def update(self, cur_ns: int, msgs_seen: int, msgs_kept: int):
        now = time.monotonic()
        if (msgs_seen % self.update_every_n) != 0 and (now - self._last_print) < self.update_period_s:
            return

        elapsed = now - self._t0
        cur_ns = int(cur_ns)

        if self.total_end_ns is not None and self.total_end_ns > self.bag_start_ns:
            denom = float(self.total_end_ns - self.bag_start_ns)
            ratio = (float(cur_ns - self.bag_start_ns) / denom) if denom > 0.0 else 0.0
            ratio = max(0.0, min(1.0, ratio))

            if ratio == self._last_ratio and (now - self._last_print) < (self.update_period_s * 2.0):
                return

            bar = self._format_bar(ratio)
            t_s = (cur_ns - self.bag_start_ns) / 1e9
            pct = 100.0 * ratio
            msg = (
                f"\r{self.label} {bar} {pct:6.2f}% "
                f"t={t_s:8.3f}s  seen={msgs_seen}  kept={msgs_kept}  elapsed={elapsed:6.1f}s"
            )
            self._last_ratio = ratio
        else:
            t_s = (cur_ns - self.bag_start_ns) / 1e9
            msg = f"\r{self.label} seen={msgs_seen} kept={msgs_kept} t={t_s:8.3f}s elapsed={elapsed:6.1f}s"

        self.stream.write(msg)
        self.stream.flush()
        self._last_print = now

    def close(self, final_ns: int, msgs_seen: int, msgs_kept: int):
        if self.total_end_ns is not None and self.total_end_ns > self.bag_start_ns:
            self.total_end_ns = max(self.total_end_ns, int(final_ns))
            self._last_ratio = -1.0
            self.update(self.total_end_ns, msgs_seen, msgs_kept)
        else:
            self.update(final_ns, msgs_seen, msgs_kept)
        self.stream.write("\n")
        self.stream.flush()


# =========================
# SERIES FINALIZATION
# =========================
def _finalize_xy(t_list, y_list, max_points: int | None) -> tuple[np.ndarray, np.ndarray]:
    if t_list is None or y_list is None:
        return np.empty((0,), dtype=np.int64), np.empty((0,), dtype=np.float64)
    if len(t_list) == 0 or len(y_list) == 0:
        return np.empty((0,), dtype=np.int64), np.empty((0,), dtype=np.float64)

    t = np.asarray(t_list, dtype=np.int64)
    y = np.asarray(y_list, dtype=np.float64)
    n = min(len(t), len(y))
    t = t[:n]
    y = y[:n]

    order = np.argsort(t, kind="mergesort")
    t = t[order]
    y = y[order]

    if max_points is not None and max_points > 0 and len(t) > max_points:
        step = int(math.ceil(len(t) / max_points))
        t = t[::step]
        y = y[::step]

    return t.astype(np.int64), y.astype(np.float64)


def _pick_primary_topic_jointed(data_dict) -> str | None:
    best_topic = None
    best_count = -1
    for topic, joints in data_dict.items():
        c = 0
        for _, fields in joints.items():
            c += len(fields.get("t", []))
        if c > best_count:
            best_count = c
            best_topic = topic
    return best_topic


def _pick_primary_topic_simple(data_dict, key: str = "t") -> str | None:
    best_topic = None
    best_count = -1
    for topic, fields in data_dict.items():
        c = len(fields.get(key, []))
        if c > best_count:
            best_count = c
            best_topic = topic
    return best_topic


def _pick_best_cmd_topic(data_dict, joints: list[str], field: str, eps: float = 1e-9) -> str | None:
    best_topic = None
    best_nonzero = -1
    best_finite = -1

    for topic, jd in data_dict.items():
        nonzero = 0
        finite = 0
        for j in joints:
            if j not in jd:
                continue
            y = np.asarray(jd[j].get(field, []), dtype=np.float64)
            if y.size == 0:
                continue
            m = np.isfinite(y)
            finite += int(np.count_nonzero(m))
            if np.any(m):
                nonzero += int(np.count_nonzero(m & (np.abs(y) > eps)))

        if nonzero > best_nonzero or (nonzero == best_nonzero and finite > best_finite):
            best_nonzero = nonzero
            best_finite = finite
            best_topic = topic

    return best_topic


# =========================
# PLOTTING HELPERS
# =========================
def _ensure_outdir(outdir: Path):
    outdir.mkdir(parents=True, exist_ok=True)


def _setup_matplotlib_defaults():
    plt.rcParams["font.size"] = 7
    plt.rcParams["axes.grid"] = True
    plt.rcParams["grid.alpha"] = 0.25
    plt.rcParams["lines.linewidth"] = 0.9


def _ordered_joints(joints: list[str]) -> list[str]:
    canon_set = set(ANYMAL_JOINT_NAMES)
    canon = [j for j in ANYMAL_JOINT_NAMES if j in joints]
    rest = sorted([j for j in joints if j not in canon_set])
    return canon + rest


def _legend_top_single_row(fig, handles, labels, *, title: str | None = None, fontsize: int = LEGEND_FONTSIZE) -> float:
    """
    Top legend with controlled wrapping (caps ncol) so it doesn't overlap/clipped when figures are short.
    Returns 'rect_top' for tight_layout(..., rect=[..., rect_top]).
    """
    if not handles:
        return 0.98

    ncol = max(1, min(len(labels), int(LEGEND_MAX_NCOL)))

    leg = fig.legend(
        handles,
        labels,
        loc="upper center",
        bbox_to_anchor=(0.5, 0.995),
        ncol=ncol,
        frameon=True,
        framealpha=float(LEGEND_FRAMEALPHA),
        fontsize=fontsize,
        borderaxespad=0.0,
        columnspacing=0.8,
        handletextpad=0.4,
        labelspacing=0.2,
        title=title,
        title_fontsize=fontsize,
    )

    fig.canvas.draw()
    renderer = fig.canvas.get_renderer()
    bbox = leg.get_window_extent(renderer=renderer).transformed(fig.transFigure.inverted())
    rect_top = max(0.0, min(0.98, bbox.y0 - 0.01))
    return rect_top


def _default_color_cycle() -> list[str]:
    try:
        cols = plt.rcParams["axes.prop_cycle"].by_key().get("color", None)
        if cols:
            return list(cols)
    except Exception:
        pass
    return [f"C{i}" for i in range(10)]


def _finite_minmax(arrs: Iterable[np.ndarray]) -> tuple[float | None, float | None]:
    vmin = None
    vmax = None
    for a in arrs:
        if a is None:
            continue
        aa = np.asarray(a, dtype=np.float64)
        if aa.size == 0:
            continue
        mask = np.isfinite(aa)
        if not np.any(mask):
            continue
        mn = float(np.min(aa[mask]))
        mx = float(np.max(aa[mask]))
        vmin = mn if vmin is None else min(vmin, mn)
        vmax = mx if vmax is None else max(vmax, mx)
    return vmin, vmax


def _auto_ylim(ax, arrs: Iterable[np.ndarray], *, pad_frac: float = 0.05, min_pad: float = 1e-6):
    vmin, vmax = _finite_minmax(arrs)
    if vmin is None or vmax is None:
        return
    if not math.isfinite(vmin) or not math.isfinite(vmax):
        return
    if vmax <= vmin:
        pad = max(min_pad, abs(vmin) * pad_frac, 1.0)
        ax.set_ylim(vmin - pad, vmax + pad)
        return
    span = vmax - vmin
    pad = max(min_pad, span * pad_frac)
    ax.set_ylim(vmin - pad, vmax + pad)


# =========================
# PLOTTING
# =========================
def _plot_joint_grid(
    out_path: Path,
    joints: list[str],
    t0_ns: int,
    est_topic: str | None,
    sea_state_topic: str | None,
    sea_cmd_topic: str | None,
    est_field: str,
    sea_state_field: str,
    sea_cmd_field: str | None,
    ylabel: str,
    legend_word: str,
):
    n = len(joints)
    if n == 0:
        return

    ncols = 3
    nrows = int(math.ceil(n / ncols))

    fig_h = max(
        float(JOINTS_FIG_MIN_HEIGHT_IN),
        float(JOINTS_FIG_BASE_HEIGHT_IN) + float(JOINTS_FIG_HEIGHT_PER_ROW_IN) * float(nrows),
    )

    fig, axes = plt.subplots(nrows=nrows, ncols=ncols, figsize=(float(FIG_WIDTH_IN), float(fig_h)), sharex=True)
    axes = np.atleast_1d(axes).ravel()

    handles = []
    labels = []

    for i, j in enumerate(joints):
        ax = axes[i]
        ax.text(0.02, 0.96, j, transform=ax.transAxes, va="top", fontsize=7, bbox=TEXT_BBOX, zorder=TEXT_ZORDER)

        y_for_ylim: list[np.ndarray] = []

        if est_topic is not None and j in est_data[est_topic]:
            t_ns, y = _finalize_xy(
                est_data[est_topic][j].get("t", []),
                est_data[est_topic][j].get(est_field, []),
                MAX_POINTS_PER_SERIES,
            )
            if len(t_ns):
                x = _ns_to_rel_s(t_ns, t0_ns)
                (h,) = ax.plot(x, y, label="Estimator")
                y_for_ylim.append(y)
                if "Estimator" not in labels:
                    handles.append(h)
                    labels.append("Estimator")

        if sea_state_topic is not None and j in sea_data[sea_state_topic]:
            t_ns_s, y_s = _finalize_xy(
                sea_data[sea_state_topic][j].get("t", []),
                sea_data[sea_state_topic][j].get(sea_state_field, []),
                MAX_POINTS_PER_SERIES,
            )
            if len(t_ns_s):
                x_s = _ns_to_rel_s(t_ns_s, t0_ns)
                (h,) = ax.plot(x_s, y_s, label="SEA")
                y_for_ylim.append(y_s)
                if "SEA" not in labels:
                    handles.append(h)
                    labels.append("SEA")

        cmd_topic = sea_cmd_topic if sea_cmd_topic is not None else sea_state_topic
        if sea_cmd_field is not None and cmd_topic is not None and j in sea_data[cmd_topic]:
            t_ns_c, y_c = _finalize_xy(
                sea_data[cmd_topic][j].get("t", []),
                sea_data[cmd_topic][j].get(sea_cmd_field, []),
                MAX_POINTS_PER_SERIES,
            )
            if len(t_ns_c):
                x_c = _ns_to_rel_s(t_ns_c, t0_ns)
                (h,) = ax.plot(x_c, y_c, label="Target", linestyle="--")
                y_for_ylim.append(y_c)
                if "Target" not in labels:
                    handles.append(h)
                    labels.append("Target")

        if i % ncols == 0:
            ax.set_ylabel(ylabel)

        _apply_xlim(ax)
        _auto_ylim(ax, y_for_ylim)

    for k in range(n, len(axes)):
        axes[k].axis("off")

    rect_top = _legend_top_single_row(fig, handles, labels, title=legend_word, fontsize=LEGEND_FONTSIZE)
    fig.supxlabel("Time[s]")
    fig.tight_layout(pad=0.18, rect=[0, 0, 1, rect_top])

    fig.savefig(out_path, dpi=DPI, bbox_inches="tight")
    plt.close(fig)


def _plot_contacts(outdir: Path, t0_ns: int, est_topic: str | None):
    if est_topic is None or est_topic not in contacts_data:
        return

    contacts = sorted(contacts_data[est_topic].keys())
    if not contacts:
        return

    fig_h = max(
        float(CONTACT_FIG_MIN_HEIGHT_IN),
        float(CONTACT_FIG_BASE_HEIGHT_IN) + float(CONTACT_FIG_HEIGHT_PER_CONTACT_IN) * float(len(contacts)),
    )

    fig, (ax0, ax1) = plt.subplots(
        nrows=2,
        ncols=1,
        figsize=(float(FIG_WIDTH_IN), float(fig_h)),
        sharex=True,
        gridspec_kw={"height_ratios": list(CONTACT_HEIGHT_RATIOS)},
    )

    colors = _default_color_cycle()
    legend_handles = []
    legend_labels = []

    # Y-axis ticks: for each contact row, show 0 and 1 (instead of contact names).
    yticks: list[float] = []
    yticklabels: list[str] = []

    for i, cname in enumerate(contacts):
        color = colors[i % len(colors)]

        t_ns, state = _finalize_xy(
            contacts_data[est_topic][cname].get("t", []),
            contacts_data[est_topic][cname].get("state", []),
            MAX_POINTS_PER_SERIES,
        )
        if len(t_ns):
            x = _ns_to_rel_s(t_ns, t0_ns)
            st = (state > 0).astype(np.float64)

            y0 = i * float(CONTACT_ROW_SPACING)
            y = y0 + float(CONTACT_STATE_AMPL) * st

            # Baseline guide for the row.
            ax0.hlines(
                y0,
                xmin=float(START_TIME_S),
                xmax=float(END_TIME_S) if END_TIME_S is not None else float(x[-1]),
                linewidth=0.5,
                alpha=0.3,
            )

            h = ax0.step(x, y, where="post", color=color, label=cname)[0]
            legend_handles.append(h)
            legend_labels.append(cname)

            # Per-row y ticks: 0 at baseline, 1 at baseline+ampl.
            yticks.extend([y0, y0 + float(CONTACT_STATE_AMPL)])
            yticklabels.extend(["0", "1"])

        t_nf, fmag = _finalize_xy(
            contacts_data[est_topic][cname].get("t", []),
            contacts_data[est_topic][cname].get("force_norm", []),
            MAX_POINTS_PER_SERIES,
        )
        if len(t_nf):
            xf = _ns_to_rel_s(t_nf, t0_ns)
            ax1.plot(xf, fmag, color=color, label="_nolegend_")

    ax0.set_ylabel("contact state")
    if yticks:
        ax0.set_yticks(yticks)
        ax0.set_yticklabels(yticklabels)
    ax0.tick_params(axis="y", labelsize=float(CONTACT_YTICK_FONTSIZE))
    ax0.set_ylim(-0.5, (len(contacts) - 1) * float(CONTACT_ROW_SPACING) + float(CONTACT_STATE_AMPL) + 0.5)
    _apply_xlim(ax0)

    ax1.set_ylabel("|force| [N]")
    ax1.set_xlabel("Time[s]")
    _apply_xlim(ax1)

    all_forces = []
    for cname in contacts:
        y = np.asarray(contacts_data[est_topic][cname].get("force_norm", []), dtype=np.float64)
        all_forces.append(y)
    _auto_ylim(ax1, all_forces, pad_frac=0.08)

    rect_top = _legend_top_single_row(fig, legend_handles, legend_labels, title="Contacts", fontsize=LEGEND_FONTSIZE)
    fig.tight_layout(pad=0.25, rect=[0, 0, 1, rect_top])
    fig.savefig(outdir / "contacts_state_and_force.pdf", dpi=DPI, bbox_inches="tight")
    plt.close(fig)


def _plot_imu_all(outdir: Path, t0_ns: int):
    out_path = outdir / "imu.pdf"

    if not imu_data:
        fig, ax = plt.subplots(figsize=(float(FIG_WIDTH_IN), max(1.8, float(IMU_FIG_HEIGHT_IN) * 0.6)))
        ax.axis("off")
        ax.text(0.01, 0.6, "No IMU topics found in bag.", transform=ax.transAxes, bbox=TEXT_BBOX, zorder=TEXT_ZORDER)
        fig.tight_layout(pad=0.2)
        fig.savefig(out_path, dpi=DPI, bbox_inches="tight")
        plt.close(fig)
        return

    topics = sorted(imu_data.keys())
    topics_with_data = [tp for tp in topics if len(imu_data[tp].get("t", [])) > 0]
    if not topics_with_data:
        fig, ax = plt.subplots(figsize=(float(FIG_WIDTH_IN), max(1.8, float(IMU_FIG_HEIGHT_IN) * 0.6)))
        ax.axis("off")
        ax.text(
            0.01,
            0.6,
            f"No IMU messages in requested window [{START_TIME_S}, {END_TIME_S if END_TIME_S is not None else 'end'}] s.",
            transform=ax.transAxes,
            bbox=TEXT_BBOX,
            zorder=TEXT_ZORDER,
        )
        fig.tight_layout(pad=0.2)
        fig.savefig(out_path, dpi=DPI, bbox_inches="tight")
        plt.close(fig)
        return

    topics_with_data = sorted(topics_with_data, key=lambda tp: (_imu_draw_priority(tp), tp))

    fig, (ax0, ax1) = plt.subplots(
        nrows=2, ncols=1, figsize=(float(FIG_WIDTH_IN), float(IMU_FIG_HEIGHT_IN)), sharex=True
    )

    colors = _default_color_cycle()
    legend_handles: list = []
    legend_labels: list[str] = []

    y_gyro_all: list[np.ndarray] = []
    y_acc_all: list[np.ndarray] = []

    g0 = float(G0_MPS2) if float(G0_MPS2) != 0.0 else 1.0

    for i, tp in enumerate(topics_with_data):
        t_list = imu_data[tp].get("t", [])
        alias = _imu_alias(tp)

        t_ns, avx = _finalize_xy(t_list, imu_data[tp].get("avx", []), MAX_POINTS_PER_SERIES)
        _, lax = _finalize_xy(t_list, imu_data[tp].get("lax", []), MAX_POINTS_PER_SERIES)
        lax_g = (lax / g0) if lax.size else lax

        if len(t_ns) == 0:
            continue

        x = _ns_to_rel_s(t_ns, t0_ns)
        color = colors[i % len(colors)]
        z = _imu_zorder(tp)

        (h,) = ax0.plot(x, avx, color=color, label=alias, zorder=z)
        ax1.plot(x, lax_g, color=color, label="_nolegend_", zorder=z)

        legend_handles.append(h)
        legend_labels.append(alias)

        y_gyro_all.append(avx)
        y_acc_all.append(lax_g)

    ax0.set_ylabel("gyro x [rad/s]")
    ax1.set_ylabel("accel x [g]")
    ax1.set_xlabel("Time[s]")
    _apply_xlim(ax0)
    _apply_xlim(ax1)

    _auto_ylim(ax0, y_gyro_all, pad_frac=0.08)
    _auto_ylim(ax1, y_acc_all, pad_frac=0.08)

    rect_top = _legend_top_single_row(fig, legend_handles, legend_labels, title="IMU", fontsize=LEGEND_FONTSIZE)
    fig.tight_layout(pad=0.25, rect=[0, 0, 1, rect_top])
    fig.savefig(out_path, dpi=DPI, bbox_inches="tight")
    plt.close(fig)


def _plot_pointstamped_timeseries(outdir: Path, t0_ns: int):
    if not point_data:
        return

    for topic, fields in point_data.items():
        t_ns, xs = _finalize_xy(fields.get("t", []), fields.get("x", []), MAX_POINTS_PER_SERIES)
        _, ys = _finalize_xy(fields.get("t", []), fields.get("y", []), MAX_POINTS_PER_SERIES)
        _, zs = _finalize_xy(fields.get("t", []), fields.get("z", []), MAX_POINTS_PER_SERIES)

        if len(t_ns) == 0:
            continue

        tt = _ns_to_rel_s(t_ns, t0_ns)

        fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(float(FIG_WIDTH_IN), 2.3))
        (h1,) = ax.plot(tt, xs, label="x")
        (h2,) = ax.plot(tt, ys, label="y")
        (h3,) = ax.plot(tt, zs, label="z")
        ax.set_xlabel("Time[s]")
        ax.set_ylabel("point")
        _apply_xlim(ax)
        _auto_ylim(ax, [xs, ys, zs], pad_frac=0.08)

        rect_top = _legend_top_single_row(
            fig, [h1, h2, h3], ["x", "y", "z"], title=_basename_topic(topic), fontsize=LEGEND_FONTSIZE
        )
        fig.tight_layout(pad=0.25, rect=[0, 0, 1, rect_top])
        fig.savefig(outdir / f"pointstamped_{_sanitize(topic)}_xyz_vs_time.pdf", dpi=DPI, bbox_inches="tight")
        plt.close(fig)


def _plot_pointstamped_xy_overview(outdir: Path):
    if not point_data:
        return

    fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(6.6, 6.0))
    any_plotted = False

    handles = []
    labels = []

    for topic, fields in point_data.items():
        _, xs = _finalize_xy(fields.get("t", []), fields.get("x", []), MAX_POINTS_PER_SERIES)
        _, ys = _finalize_xy(fields.get("t", []), fields.get("y", []), MAX_POINTS_PER_SERIES)
        if len(xs) == 0 or len(ys) == 0:
            continue
        lab = _sanitize(_basename_topic(topic))
        (h,) = ax.plot(xs, ys, label=lab)
        handles.append(h)
        labels.append(lab)
        any_plotted = True

    if not any_plotted:
        plt.close(fig)
        return

    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.axis("equal")

    rect_top = _legend_top_single_row(fig, handles, labels, title="PointStamped XY", fontsize=LEGEND_FONTSIZE)
    fig.tight_layout(pad=0.25, rect=[0, 0, 1, rect_top])
    fig.savefig(outdir / "pointstamped_xy_overview.pdf", dpi=DPI, bbox_inches="tight")
    plt.close(fig)


def _plot_twist_cmd_vs_odom(outdir: Path, t0_ns: int, cmd_topic: str | None, odom_topic: str | None):
    """
    Compare geometry_msgs/TwistStamped (command) against nav_msgs/Odometry.twist (estimate).
    Produces 2x3 grid:
      Row 1: linear (x,y,z) [m/s]
      Row 2: angular (x,y,z) [rad/s]
    Each subplot overlays: command (dashed) and odom (solid).
    """
    if cmd_topic is None and odom_topic is None:
        return

    have_cmd = cmd_topic is not None and cmd_topic in twist_cmd_data and len(twist_cmd_data[cmd_topic].get("t", [])) > 0
    have_odom = (
        odom_topic is not None and odom_topic in odom_twist_data and len(odom_twist_data[odom_topic].get("t", [])) > 0
    )
    if not have_cmd and not have_odom:
        return

    out_path = outdir / "twist_command_vs_odometry.pdf"

    fig, axes = plt.subplots(nrows=2, ncols=3, figsize=(float(FIG_WIDTH_IN), float(TWIST_FIG_HEIGHT_IN)), sharex=True)
    axes = np.asarray(axes)

    comps = [("lin_x", "x"), ("lin_y", "y"), ("lin_z", "z")]
    acomps = [("ang_x", "x"), ("ang_y", "y"), ("ang_z", "z")]

    cmd_series = {}
    odom_series = {}

    if have_cmd:
        t_list = twist_cmd_data[cmd_topic].get("t", [])
        t_ns, _ = _finalize_xy(t_list, twist_cmd_data[cmd_topic].get("lin_x", []), MAX_POINTS_PER_SERIES)
        cmd_x = _ns_to_rel_s(t_ns, t0_ns) if len(t_ns) else np.empty((0,), dtype=np.float64)
        for k in ("lin_x", "lin_y", "lin_z", "ang_x", "ang_y", "ang_z"):
            _, y = _finalize_xy(t_list, twist_cmd_data[cmd_topic].get(k, []), MAX_POINTS_PER_SERIES)
            cmd_series[k] = y
        cmd_series["_x"] = cmd_x

    if have_odom:
        t_list = odom_twist_data[odom_topic].get("t", [])
        t_ns, _ = _finalize_xy(t_list, odom_twist_data[odom_topic].get("lin_x", []), MAX_POINTS_PER_SERIES)
        odom_x = _ns_to_rel_s(t_ns, t0_ns) if len(t_ns) else np.empty((0,), dtype=np.float64)
        for k in ("lin_x", "lin_y", "lin_z", "ang_x", "ang_y", "ang_z"):
            _, y = _finalize_xy(t_list, odom_twist_data[odom_topic].get(k, []), MAX_POINTS_PER_SERIES)
            odom_series[k] = y
        odom_series["_x"] = odom_x

    legend_handles = []
    legend_labels = []

    for c, (key, axis_name) in enumerate(comps):
        ax = axes[0, c]
        ax.text(
            0.02,
            0.96,
            f"linear {axis_name}",
            transform=ax.transAxes,
            va="top",
            fontsize=7,
            bbox=TEXT_BBOX,
            zorder=TEXT_ZORDER,
        )

        y_for_ylim = []
        if have_odom and odom_series["_x"].size and odom_series[key].size:
            (h1,) = ax.plot(odom_series["_x"], odom_series[key], label="Odometry")
            y_for_ylim.append(odom_series[key])
            if "Odometry" not in legend_labels:
                legend_handles.append(h1)
                legend_labels.append("Odometry")

        if have_cmd and cmd_series["_x"].size and cmd_series[key].size:
            (h2,) = ax.plot(cmd_series["_x"], cmd_series[key], linestyle="--", label="Command")
            y_for_ylim.append(cmd_series[key])
            if "Command" not in legend_labels:
                legend_handles.append(h2)
                legend_labels.append("Command")

        if c == 0:
            ax.set_ylabel("linear [m/s]")
        _apply_xlim(ax)
        _auto_ylim(ax, y_for_ylim, pad_frac=0.08)

    for c, (key, axis_name) in enumerate(acomps):
        ax = axes[1, c]
        ax.text(
            0.02,
            0.96,
            f"angular {axis_name}",
            transform=ax.transAxes,
            va="top",
            fontsize=7,
            bbox=TEXT_BBOX,
            zorder=TEXT_ZORDER,
        )

        y_for_ylim = []
        if have_odom and odom_series["_x"].size and odom_series[key].size:
            ax.plot(odom_series["_x"], odom_series[key], label="_nolegend_")
            y_for_ylim.append(odom_series[key])

        if have_cmd and cmd_series["_x"].size and cmd_series[key].size:
            ax.plot(cmd_series["_x"], cmd_series[key], linestyle="--", label="_nolegend_")
            y_for_ylim.append(cmd_series[key])

        if c == 0:
            ax.set_ylabel("angular [rad/s]")
        _apply_xlim(ax)
        _auto_ylim(ax, y_for_ylim, pad_frac=0.08)

    rect_top = _legend_top_single_row(fig, legend_handles, legend_labels, title="Twist", fontsize=LEGEND_FONTSIZE)
    fig.supxlabel("Time[s]")
    fig.tight_layout(pad=0.16, rect=[0, 0, 1, rect_top])
    fig.savefig(out_path, dpi=DPI, bbox_inches="tight")
    plt.close(fig)


# =========================
# BAG READING / PARSING
# =========================
def _get_bag_start_ns(reader: AnyReader) -> int:
    st = getattr(reader, "start_time", None)
    if isinstance(st, (int, np.integer)):
        return int(st)
    for _, log_ns, _ in reader.messages():
        return int(log_ns)
    raise RuntimeError("Bag appears empty; cannot determine start time.")


def _get_bag_end_ns(reader: AnyReader) -> int | None:
    et = getattr(reader, "end_time", None)
    if isinstance(et, (int, np.integer)):
        return int(et)
    return None


def _read_bag(bag_path: Path) -> int:
    if not bag_path.is_file():
        raise FileNotFoundError(f"Bag not found: {bag_path}")

    start_s = float(START_TIME_S)
    if start_s < 0.0:
        start_s = 0.0

    if END_TIME_S is not None:
        end_s = float(END_TIME_S)
        if end_s < start_s:
            end_s = start_s
    else:
        end_s = None

    with AnyReader([bag_path]) as reader:
        bag_start_ns = _get_bag_start_ns(reader)
        bag_end_ns = _get_bag_end_ns(reader)

        window_start_ns = bag_start_ns + _s_to_ns(start_s)
        if end_s is not None:
            window_end_ns = bag_start_ns + _s_to_ns(end_s)
        else:
            window_end_ns = bag_end_ns if bag_end_ns is not None else (2**63 - 1)

        if bag_end_ns is not None:
            window_end_ns = min(window_end_ns, bag_end_ns)

        conns = list(reader.connections)

        anymal_conns = [c for c in conns if _is_type(c.msgtype, T_ANYMAL_STATE)]
        sea_conns = [c for c in conns if _is_type(c.msgtype, T_SEA_READINGS)]
        imu_conns = [c for c in conns if _is_type(c.msgtype, T_IMU)]
        pnt_conns = [c for c in conns if _is_type(c.msgtype, T_POINTSTAMPED)]
        tws_conns = [c for c in conns if _is_type(c.msgtype, T_TWISTSTAMPED)]
        odo_conns = [c for c in conns if _is_type(c.msgtype, T_ODOMETRY)]

        wanted = anymal_conns + sea_conns + imu_conns + pnt_conns + tws_conns + odo_conns
        if not wanted:
            seen = sorted(set(_norm_type(c.msgtype) for c in conns))
            raise RuntimeError(
                "No supported topics found.\n"
                f"Looking for: {T_ANYMAL_STATE}, {T_SEA_READINGS}, {T_IMU}, {T_POINTSTAMPED}, {T_TWISTSTAMPED}, {T_ODOMETRY}\n"
                f"Seen msgtypes:\n  " + "\n  ".join(seen)
            )

        total_end_ns = (
            window_end_ns if window_end_ns < (2**63 - 1) else (bag_end_ns if bag_end_ns is not None else None)
        )
        pbar = _ProgressBar(
            label="Reading bag:",
            bag_start_ns=bag_start_ns,
            total_end_ns=total_end_ns,
            update_period_s=PROGRESS_UPDATE_PERIOD_S,
            update_every_n=PROGRESS_UPDATE_EVERY_N_MSG,
        )

        msgs_seen = 0
        msgs_kept = 0
        last_ns = bag_start_ns

        for conn, log_ns, raw in reader.messages(connections=wanted):
            log_ns_i = int(log_ns)
            msgs_seen += 1
            last_ns = log_ns_i
            pbar.update(log_ns_i, msgs_seen, msgs_kept)

            if log_ns_i < window_start_ns:
                continue
            if log_ns_i > window_end_ns:
                break

            topic = conn.topic
            msgtype = _norm_type(conn.msgtype)

            try:
                msg = reader.deserialize(raw, conn.msgtype)
            except Exception:
                continue

            ns = log_ns_i
            msgs_kept += 1
            _track_time(ns)

            if _is_type(msgtype, T_ANYMAL_STATE):
                js = getattr(msg, "joints", None)
                if js is not None:
                    names = _as_list(getattr(js, "name", None))
                    pos = _as_np_float(getattr(js, "position", None))
                    vel = _as_np_float(getattr(js, "velocity", None))
                    eff = _as_np_float(getattr(js, "effort", None))
                    acc = _as_np_float(getattr(js, "acceleration", None))

                    n_num = min(len(pos), len(vel), len(eff))
                    if n_num > 0:
                        if len(names) < n_num:
                            names = names + [""] * (n_num - len(names))
                        names = names[:n_num]

                        if len(acc) < n_num:
                            if len(acc) == 0:
                                acc = np.full((n_num,), np.nan, dtype=np.float64)
                            else:
                                acc = np.concatenate(
                                    [acc, np.full((n_num - len(acc),), np.nan, dtype=np.float64)], axis=0
                                )
                        else:
                            acc = acc[:n_num]

                        for i in range(n_num):
                            jname = _normalize_joint_name(str(names[i]) if names[i] is not None else "", i)
                            est_data[topic][jname]["t"].append(ns)
                            est_data[topic][jname]["pos"].append(float(pos[i]))
                            est_data[topic][jname]["vel"].append(float(vel[i]))
                            est_data[topic][jname]["eff"].append(float(eff[i]))
                            est_data[topic][jname]["acc"].append(float(acc[i]))

                c_list = getattr(msg, "contacts", None)
                if isinstance(c_list, (list, tuple)):
                    for c in c_list:
                        cname = _sanitize(getattr(c, "name", "contact"))
                        st = int(getattr(c, "state", 0) or 0)

                        contacts_data[topic][cname]["t"].append(ns)
                        contacts_data[topic][cname]["state"].append(float(st))

                        w = getattr(c, "wrench", None)
                        contacts_data[topic][cname]["force_norm"].append(
                            _force_norm_from_wrench(w) if w is not None else float("nan")
                        )

            elif _is_type(msgtype, T_SEA_READINGS):
                readings = getattr(msg, "readings", None)
                if readings is None:
                    continue

                for idx, rd in enumerate(readings):
                    st = getattr(rd, "state", None)
                    cmd = getattr(rd, "commanded", None)

                    name = ""
                    if st is not None:
                        name = str(getattr(st, "name", "") or "")
                    if not name and cmd is not None:
                        name = str(getattr(cmd, "name", "") or "")
                    jname = _normalize_joint_name(name, idx)

                    sea_data[topic][jname]["t"].append(ns)

                    if st is None:
                        sea_data[topic][jname]["pos"].append(float("nan"))
                        sea_data[topic][jname]["vel"].append(float("nan"))
                        sea_data[topic][jname]["eff"].append(float("nan"))
                        sea_data[topic][jname]["cur"].append(float("nan"))
                        sea_data[topic][jname]["acc"].append(float("nan"))
                    else:
                        sea_data[topic][jname]["pos"].append(_safe_float(getattr(st, "joint_position", np.nan)))
                        sea_data[topic][jname]["vel"].append(_safe_float(getattr(st, "joint_velocity", np.nan)))
                        sea_data[topic][jname]["eff"].append(_safe_float(getattr(st, "joint_torque", np.nan)))
                        sea_data[topic][jname]["cur"].append(_safe_float(getattr(st, "current", np.nan)))
                        sea_data[topic][jname]["acc"].append(_safe_float(getattr(st, "joint_acceleration", np.nan)))

                    if cmd is None:
                        sea_data[topic][jname]["cmd_pos"].append(float("nan"))
                        sea_data[topic][jname]["cmd_vel"].append(float("nan"))
                        sea_data[topic][jname]["cmd_eff"].append(float("nan"))
                        sea_data[topic][jname]["cmd_cur"].append(float("nan"))
                    else:
                        sea_data[topic][jname]["cmd_pos"].append(
                            _get_attr_any(
                                cmd,
                                ["position", "joint_position", "joint_position_desired", "desired_position", "pos"],
                                np.nan,
                            )
                        )
                        sea_data[topic][jname]["cmd_vel"].append(
                            _get_attr_any(
                                cmd,
                                [
                                    "velocity",
                                    "joint_velocity",
                                    "joint_velocity_desired",
                                    "joint_velocity_command",
                                    "desired_velocity",
                                    "vel",
                                ],
                                np.nan,
                            )
                        )
                        sea_data[topic][jname]["cmd_eff"].append(
                            _get_attr_any(cmd, ["joint_torque", "torque", "effort", "joint_effort", "tau"], np.nan)
                        )
                        sea_data[topic][jname]["cmd_cur"].append(
                            _get_attr_any(cmd, ["current", "motor_current", "iq"], np.nan)
                        )

            elif _is_type(msgtype, T_IMU):
                av = getattr(msg, "angular_velocity", None)
                la = getattr(msg, "linear_acceleration", None)

                imu_data[topic]["t"].append(ns)
                imu_data[topic]["avx"].append(_safe_float(getattr(av, "x", np.nan)))
                imu_data[topic]["avy"].append(_safe_float(getattr(av, "y", np.nan)))
                imu_data[topic]["avz"].append(_safe_float(getattr(av, "z", np.nan)))
                imu_data[topic]["lax"].append(_safe_float(getattr(la, "x", np.nan)))
                imu_data[topic]["lay"].append(_safe_float(getattr(la, "y", np.nan)))
                imu_data[topic]["laz"].append(_safe_float(getattr(la, "z", np.nan)))

            elif _is_type(msgtype, T_POINTSTAMPED):
                pt = getattr(msg, "point", None)
                if pt is None:
                    continue
                point_data[topic]["t"].append(ns)
                point_data[topic]["x"].append(_safe_float(getattr(pt, "x", np.nan)))
                point_data[topic]["y"].append(_safe_float(getattr(pt, "y", np.nan)))
                point_data[topic]["z"].append(_safe_float(getattr(pt, "z", np.nan)))

            elif _is_type(msgtype, T_TWISTSTAMPED):
                tw = getattr(msg, "twist", None)
                if tw is None:
                    continue
                lin = getattr(tw, "linear", None)
                ang = getattr(tw, "angular", None)

                twist_cmd_data[topic]["t"].append(ns)
                twist_cmd_data[topic]["lin_x"].append(_safe_float(getattr(lin, "x", np.nan)))
                twist_cmd_data[topic]["lin_y"].append(_safe_float(getattr(lin, "y", np.nan)))
                twist_cmd_data[topic]["lin_z"].append(_safe_float(getattr(lin, "z", np.nan)))
                twist_cmd_data[topic]["ang_x"].append(_safe_float(getattr(ang, "x", np.nan)))
                twist_cmd_data[topic]["ang_y"].append(_safe_float(getattr(ang, "y", np.nan)))
                twist_cmd_data[topic]["ang_z"].append(_safe_float(getattr(ang, "z", np.nan)))

            elif _is_type(msgtype, T_ODOMETRY):
                tw = getattr(msg, "twist", None)
                tw2 = getattr(tw, "twist", None) if tw is not None else None
                if tw2 is None:
                    continue
                lin = getattr(tw2, "linear", None)
                ang = getattr(tw2, "angular", None)

                odom_twist_data[topic]["t"].append(ns)
                odom_twist_data[topic]["lin_x"].append(_safe_float(getattr(lin, "x", np.nan)))
                odom_twist_data[topic]["lin_y"].append(_safe_float(getattr(lin, "y", np.nan)))
                odom_twist_data[topic]["lin_z"].append(_safe_float(getattr(lin, "z", np.nan)))
                odom_twist_data[topic]["ang_x"].append(_safe_float(getattr(ang, "x", np.nan)))
                odom_twist_data[topic]["ang_y"].append(_safe_float(getattr(ang, "y", np.nan)))
                odom_twist_data[topic]["ang_z"].append(_safe_float(getattr(ang, "z", np.nan)))

        pbar.close(final_ns=last_ns, msgs_seen=msgs_seen, msgs_kept=msgs_kept)
        return bag_start_ns


# =========================
# MAIN
# =========================
def _run_plots():
    _ensure_outdir(OUTPUT_DIR)
    _setup_matplotlib_defaults()

    bag_start_ns = _read_bag(BAG_PATH)

    if global_min_ns is None:
        raise RuntimeError("No messages collected in the requested time window (or bag unsupported).")

    t0_ns = int(bag_start_ns)

    est_topic = _pick_primary_topic_jointed(est_data)
    sea_state_topic = _pick_primary_topic_jointed(sea_data)

    joint_set = set()
    if est_topic is not None:
        joint_set |= set(est_data[est_topic].keys())
    for tp in sea_data.keys():
        joint_set |= set(sea_data[tp].keys())

    joints_all = _ordered_joints(sorted(joint_set))
    joints = _filter_joints_by_leg(joints_all, ONLY_LEG)

    cmd_pos_topic = _pick_best_cmd_topic(sea_data, joints, "cmd_pos") if joints else None
    cmd_vel_topic = _pick_best_cmd_topic(sea_data, joints, "cmd_vel") if joints else None
    cmd_eff_topic = _pick_best_cmd_topic(sea_data, joints, "cmd_eff") if joints else None

    if joints:
        _plot_joint_grid(
            OUTPUT_DIR / "joints_position.pdf",
            joints=joints,
            t0_ns=t0_ns,
            est_topic=est_topic,
            sea_state_topic=sea_state_topic,
            sea_cmd_topic=cmd_pos_topic,
            est_field="pos",
            sea_state_field="pos",
            sea_cmd_field="cmd_pos",
            ylabel="position [rad]",
            legend_word="Position",
        )

        _plot_joint_grid(
            OUTPUT_DIR / "joints_velocity.pdf",
            joints=joints,
            t0_ns=t0_ns,
            est_topic=est_topic,
            sea_state_topic=sea_state_topic,
            sea_cmd_topic=cmd_vel_topic,
            est_field="vel",
            sea_state_field="vel",
            sea_cmd_field="cmd_vel",
            ylabel="velocity [rad/s]",
            legend_word="Velocity",
        )

        _plot_joint_grid(
            OUTPUT_DIR / "joints_effort.pdf",
            joints=joints,
            t0_ns=t0_ns,
            est_topic=est_topic,
            sea_state_topic=sea_state_topic,
            sea_cmd_topic=cmd_eff_topic,
            est_field="eff",
            sea_state_field="eff",
            sea_cmd_field="cmd_eff",
            ylabel="torque [Nm]",
            legend_word="Torque",
        )

        _plot_joint_grid(
            OUTPUT_DIR / "joints_acceleration.pdf",
            joints=joints,
            t0_ns=t0_ns,
            est_topic=est_topic,
            sea_state_topic=sea_state_topic,
            sea_cmd_topic=None,
            est_field="acc",
            sea_state_field="acc",
            sea_cmd_field=None,
            ylabel="acc [rad/s^2]",
            legend_word="Acceleration",
        )

    _plot_contacts(OUTPUT_DIR, t0_ns, est_topic)
    _plot_imu_all(OUTPUT_DIR, t0_ns)
    _plot_pointstamped_timeseries(OUTPUT_DIR, t0_ns)
    _plot_pointstamped_xy_overview(OUTPUT_DIR)

    cmd_tw_topic = (
        TWIST_CMD_TOPIC_HINT if (TWIST_CMD_TOPIC_HINT in twist_cmd_data) else _pick_primary_topic_simple(twist_cmd_data)
    )
    odom_topic = (
        ODOM_TOPIC_HINT if (ODOM_TOPIC_HINT in odom_twist_data) else _pick_primary_topic_simple(odom_twist_data)
    )
    _plot_twist_cmd_vs_odom(OUTPUT_DIR, t0_ns, cmd_tw_topic, odom_topic)

    if PLOT_PER_JOINT_FIGS and joints:
        for j in joints:
            _plot_joint_grid(
                OUTPUT_DIR / f"joint_{j}_position.pdf",
                joints=[j],
                t0_ns=t0_ns,
                est_topic=est_topic,
                sea_state_topic=sea_state_topic,
                sea_cmd_topic=cmd_pos_topic,
                est_field="pos",
                sea_state_field="pos",
                sea_cmd_field="cmd_pos",
                ylabel="position [rad]",
                legend_word="Position",
            )

    print(f"Saved plots to: {OUTPUT_DIR}")
    print(f"Time window: [{START_TIME_S}, {END_TIME_S if END_TIME_S is not None else 'end'}] s since bag start")
    print(f"Joint filter ONLY_LEG: {ONLY_LEG}")
    print(f"AnymalState chosen topic: {est_topic}")
    print(f"SEA state topic:          {sea_state_topic}")
    print(f"SEA cmd_pos topic:        {cmd_pos_topic}")
    print(f"SEA cmd_vel topic:        {cmd_vel_topic}")
    print(f"SEA cmd_eff topic:        {cmd_eff_topic}")
    if imu_data:
        print("IMU topics (with aliases):")
        for tp in sorted(imu_data.keys()):
            print(f"  {tp}  ->  {_imu_alias(tp)}  (n={len(imu_data[tp].get('t', []))})")
    if twist_cmd_data:
        print("TwistStamped topics:")
        for tp in sorted(twist_cmd_data.keys()):
            print(f"  {tp}  (n={len(twist_cmd_data[tp].get('t', []))})")
    if odom_twist_data:
        print("Odometry topics:")
        for tp in sorted(odom_twist_data.keys()):
            print(f"  {tp}  (n={len(odom_twist_data[tp].get('t', []))})")
    if point_data:
        print("PointStamped topics:")
        for tp in sorted(point_data.keys()):
            print(f"  {tp}")
    print("Files:")
    for p in sorted(OUTPUT_DIR.glob("*.pdf")):
        print(f"  {p.name}")


def main(argv=None):
    import argparse
    from box_auto.utils.atomic_file import atomic_directory

    global BAG_PATH, OUTPUT_DIR, START_TIME_S, END_TIME_S
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--start", type=float, default=0.0, help="Seconds after bag start")
    parser.add_argument("--end", type=float, default=None)
    args = parser.parse_args(argv)
    if not math.isfinite(args.start) or args.start < 0:
        parser.error("start must be finite and nonnegative")
    if args.end is not None and (not math.isfinite(args.end) or args.end <= args.start):
        parser.error("end must be finite and later than start")
    BAG_PATH = args.bag.resolve(strict=True)
    START_TIME_S, END_TIME_S = args.start, args.end
    with atomic_directory(args.output_dir) as pending:
        OUTPUT_DIR = pending
        _run_plots()
        if not list(pending.glob("*.pdf")):
            raise RuntimeError("No plots were produced")
    print(f"Published plots: {args.output_dir.resolve()}")


if __name__ == "__main__":
    main()
