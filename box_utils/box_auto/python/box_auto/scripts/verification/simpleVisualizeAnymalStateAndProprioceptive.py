#!/usr/bin/env python3
"""
Read a ROS1 .bag and generate ONLY:
  - overview_lf_hfe_contacts_imu_vertical.pdf

Overview figure (stacked):
  - LF_HFE position (Estimator + SEA + Target if available)
  - Contacts: state (per contact) + force norm
  - IMU (/anymal/imu): gyro x + accel x (in g)

Dependencies: rosbags, numpy, matplotlib
"""

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

# Time window relative to bag start time (seconds).
START_TIME_S = 15.0
END_TIME_S = 20.0  # set to None to plot until end of bag

# Optional plot decimation (points per series). Set to None to disable.
MAX_POINTS_PER_SERIES = 20000

# IMU accel plotted in g's (divide by standard gravity)
G0_MPS2 = 9.8065

# Contact visualization spacing (avoid overlaps)
CONTACT_ROW_SPACING = 2.0  # vertical spacing between contacts in the state subplot
CONTACT_STATE_AMPL = 0.8  # state amplitude within a row (must be < CONTACT_ROW_SPACING)

# -------------------------
# FIGURE SIZING (ADJUSTABLE)
# -------------------------
FIG_WIDTH_IN = 4.5
IMU_FIG_HEIGHT_IN = 2.5

CONTACT_FIG_MIN_HEIGHT_IN = 2.5
CONTACT_FIG_BASE_HEIGHT_IN = 1.5
CONTACT_FIG_HEIGHT_PER_CONTACT_IN = 0.18
CONTACT_HEIGHT_RATIOS = (1.1, 0.9)

LEGEND_MAX_NCOL = 6
LEGEND_FONTSIZE = 6
CONTACT_YTICK_FONTSIZE = 6

TEXT_BBOX = dict(facecolor="white", edgecolor="none", alpha=1.0, boxstyle="square,pad=0.15")
TEXT_ZORDER = 20
LEGEND_FRAMEALPHA = 1.0

OVERVIEW_DPI = 1200
OVERVIEW_JOINT_HEIGHT_IN = 0.85  # height of the top (LF_HFE) subplot in inches

# Progress bar update throttling
PROGRESS_UPDATE_PERIOD_S = 0.10
PROGRESS_UPDATE_EVERY_N_MSG = 200

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
# SEQ HELPERS
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


# =========================
# DATA STRUCTURES (only what we need)
# =========================
# est_data[topic][joint]["t"/"pos"]
est_data = defaultdict(lambda: defaultdict(lambda: defaultdict(list)))
# sea_data[topic][joint]["t"/"pos"/"cmd_pos"]
sea_data = defaultdict(lambda: defaultdict(lambda: defaultdict(list)))
# contacts_data[topic][contact]["t"/"state"/"force_norm"]
contacts_data = defaultdict(lambda: defaultdict(lambda: defaultdict(list)))
# imu_data[topic]["t"/"avx"/"lax"]
imu_data = defaultdict(lambda: defaultdict(list))

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


def _pick_best_cmd_topic_for_joint(data_dict, joint: str, field: str, eps: float = 1e-9) -> str | None:
    best_topic = None
    best_nonzero = -1
    best_finite = -1

    for topic, jd in data_dict.items():
        if joint not in jd:
            continue
        y = np.asarray(jd[joint].get(field, []), dtype=np.float64)
        if y.size == 0:
            continue
        m = np.isfinite(y)
        finite = int(np.count_nonzero(m))
        nonzero = int(np.count_nonzero(m & (np.abs(y) > eps))) if finite else 0

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


def _force_scientific_yaxis(ax):
    ax.ticklabel_format(axis="y", style="sci", scilimits=(0, 0), useMathText=True)
    try:
        ax.yaxis.get_offset_text().set_fontsize(float(LEGEND_FONTSIZE))
    except Exception:
        pass


gigachad_colors = [
    (0.368, 0.507, 0.71),
    (0.881, 0.611, 0.142),
    (0.923, 0.386, 0.209),
    (0.56, 0.692, 0.195),
    (0.528, 0.471, 0.701),
    (0.772, 0.432, 0.102),
    (0.572, 0.586, 0.0),
]


class _ColorPool:
    """Deterministic stream of non-repeating colors (RGB tuples)."""

    def __init__(self, *, s: float = 0.85, v: float = 0.85):
        self._i = 0
        self._colors = list(gigachad_colors)

    def next(self) -> tuple[float, float, float]:
        c = self._colors[self._i % len(self._colors)]
        self._i += 1
        return (float(c[0]), float(c[1]), float(c[2]))


# =========================
# OVERVIEW PLOT ONLY
# =========================
def _plot_overview_vertical(
    outdir: Path,
    t0_ns: int,
    est_topic: str | None,
    sea_state_topic: str | None,
    cmd_pos_topic: str | None,
):
    out_path = outdir / "overview_lf_hfe_contacts_imu_vertical.pdf"
    joint_name = "LF_HFE"
    imu_topic = "/anymal/imu"

    joint_h = float(OVERVIEW_JOINT_HEIGHT_IN)

    n_contacts = 0
    if est_topic is not None and est_topic in contacts_data:
        n_contacts = len(contacts_data[est_topic].keys())

    contact_h = max(
        float(CONTACT_FIG_MIN_HEIGHT_IN),
        float(CONTACT_FIG_BASE_HEIGHT_IN) + float(CONTACT_FIG_HEIGHT_PER_CONTACT_IN) * float(n_contacts),
    )
    imu_h = float(IMU_FIG_HEIGHT_IN)

    fig_h = float(joint_h) + float(contact_h) + float(imu_h)
    fig = plt.figure(figsize=(float(FIG_WIDTH_IN), float(fig_h)))

    gs = fig.add_gridspec(
        nrows=3,
        ncols=1,
        height_ratios=[joint_h, contact_h, imu_h],
        hspace=0.22,
    )

    axj = fig.add_subplot(gs[0, 0])

    gsc = gs[1, 0].subgridspec(2, 1, height_ratios=list(CONTACT_HEIGHT_RATIOS), hspace=0.08)
    axc0 = fig.add_subplot(gsc[0, 0], sharex=axj)
    axc1 = fig.add_subplot(gsc[1, 0], sharex=axj)

    gsi = gs[2, 0].subgridspec(2, 1, hspace=0.08)
    axi0 = fig.add_subplot(gsi[0, 0], sharex=axj)
    axi1 = fig.add_subplot(gsi[1, 0], sharex=axj)

    # Ensure every plotted data series line uses a unique color (no reuse).
    cpool = _ColorPool()

    # Collect legend entries globally and place one standalone legend at the top.
    legend_handles: list = []
    legend_labels: list[str] = []

    def _legend_add(handle, label: str):
        if handle is None:
            return
        lab = str(label or "").strip()
        if not lab or lab == "_nolegend_":
            return
        legend_handles.append(handle)
        legend_labels.append(lab)

    # ---- LF_HFE position ----
    yj_for_ylim: list[np.ndarray] = []
    axj.text(0.02, 0.96, joint_name, transform=axj.transAxes, va="top", fontsize=7, bbox=TEXT_BBOX, zorder=TEXT_ZORDER)

    # (Estimator line removed)

    if sea_state_topic is not None and joint_name in sea_data.get(sea_state_topic, {}):
        t_ns_s, y_s = _finalize_xy(
            sea_data[sea_state_topic][joint_name].get("t", []),
            sea_data[sea_state_topic][joint_name].get("pos", []),
            MAX_POINTS_PER_SERIES,
        )
        if len(t_ns_s):
            x_s = _ns_to_rel_s(t_ns_s, t0_ns)
            (h,) = axj.plot(x_s, y_s, label="SEA", color=cpool.next())
            _legend_add(h, "SEA")
            yj_for_ylim.append(y_s)

    cmd_topic = cmd_pos_topic if cmd_pos_topic is not None else sea_state_topic
    if cmd_topic is not None and joint_name in sea_data.get(cmd_topic, {}):
        t_ns_c, y_c = _finalize_xy(
            sea_data[cmd_topic][joint_name].get("t", []),
            sea_data[cmd_topic][joint_name].get("cmd_pos", []),
            MAX_POINTS_PER_SERIES,
        )
        if len(t_ns_c):
            x_c = _ns_to_rel_s(t_ns_c, t0_ns)
            (h,) = axj.plot(x_c, y_c, label="Target", linestyle="--", color=cpool.next())
            _legend_add(h, "Target")
            yj_for_ylim.append(y_c)

    axj.set_ylabel("position [rad]")
    _apply_xlim(axj)
    _auto_ylim(axj, yj_for_ylim, pad_frac=0.08)

    # ---- Contacts ----
    if est_topic is None or est_topic not in contacts_data or len(contacts_data[est_topic].keys()) == 0:
        axc0.axis("off")
        axc1.axis("off")
        axc0.text(0.01, 0.6, "No contacts found.", transform=axc0.transAxes, bbox=TEXT_BBOX, zorder=TEXT_ZORDER)
    else:
        contacts = sorted(contacts_data[est_topic].keys())

        # One persistent color per drive/contact, reused in BOTH subplots.
        contact_colors: dict[str, tuple[float, float, float]] = {}

        yticks: list[float] = []
        yticklabels: list[str] = []
        all_forces: list[np.ndarray] = []

        for i, cname in enumerate(contacts):
            # Assign exactly one color per drive, then reuse it for state and force.
            if cname not in contact_colors:
                contact_colors[cname] = cpool.next()
            color_drive = contact_colors[cname]

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

                # Guide line (neutral gray; not part of the "unique series colors").
                axc0.hlines(
                    y0,
                    xmin=float(START_TIME_S),
                    xmax=float(END_TIME_S) if END_TIME_S is not None else float(x[-1]),
                    linewidth=0.5,
                    alpha=0.3,
                    color="0.5",
                )

                h = axc0.step(x, y, where="post", color=color_drive, label=cname)[0]
                _legend_add(h, cname)

                yticks.extend([y0, y0 + float(CONTACT_STATE_AMPL)])
                yticklabels.extend(["0", "1"])

            t_nf, fmag = _finalize_xy(
                contacts_data[est_topic][cname].get("t", []),
                contacts_data[est_topic][cname].get("force_norm", []),
                MAX_POINTS_PER_SERIES,
            )
            if len(t_nf):
                xf = _ns_to_rel_s(t_nf, t0_ns)
                axc1.plot(xf, fmag, color=color_drive, label="_nolegend_")
                all_forces.append(fmag)

        axc0.set_ylabel("contact state")
        if yticks:
            axc0.set_yticks(yticks)
            axc0.set_yticklabels(yticklabels)
        axc0.tick_params(axis="y", labelsize=float(CONTACT_YTICK_FONTSIZE))
        axc0.set_ylim(-0.5, (len(contacts) - 1) * float(CONTACT_ROW_SPACING) + float(CONTACT_STATE_AMPL) + 0.5)

        axc1.set_ylabel("force [N]")
        _auto_ylim(axc1, all_forces, pad_frac=0.08)
        # _force_scientific_yaxis(axc1)

    # ---- IMU (/anymal/imu) ----
    if imu_topic not in imu_data or len(imu_data[imu_topic].get("t", [])) == 0:
        axi0.axis("off")
        axi1.axis("off")
        msg = "No /anymal/imu messages in window." if imu_topic in imu_data else "No /anymal/imu topic found."
        axi0.text(0.01, 0.6, msg, transform=axi0.transAxes, bbox=TEXT_BBOX, zorder=TEXT_ZORDER)
    else:
        t_list = imu_data[imu_topic].get("t", [])
        t_ns, avx = _finalize_xy(t_list, imu_data[imu_topic].get("avx", []), MAX_POINTS_PER_SERIES)
        _, lax = _finalize_xy(t_list, imu_data[imu_topic].get("lax", []), MAX_POINTS_PER_SERIES)

        g0 = float(G0_MPS2) if float(G0_MPS2) != 0.0 else 1.0
        lax_g = (lax / g0) if lax.size else lax

        if len(t_ns):
            x = _ns_to_rel_s(t_ns, t0_ns)
            (h0,) = axi0.plot(x, avx, label="anymal_imu", color=cpool.next())
            axi1.plot(x, lax_g, label="_nolegend_", color=cpool.next())
            _legend_add(h0, "anymal_imu")

        axi0.set_ylabel("gyro x [rad/s]")
        axi1.set_ylabel("accel x [g]")
        _auto_ylim(axi0, [avx], pad_frac=0.08)
        _auto_ylim(axi1, [lax_g], pad_frac=0.08)

    fig.supxlabel("Time[s]")
    for ax in (axj, axc0, axc1, axi0):
        ax.tick_params(labelbottom=False)

    # ---- Single standalone legend at the top (deduplicated) ----
    if legend_labels:
        uniq: dict[str, object] = {}
        for h, lab in zip(legend_handles, legend_labels):
            if lab not in uniq:
                uniq[lab] = h
        handles = list(uniq.values())
        labels = list(uniq.keys())

        ncol = max(1, min(len(labels), int(LEGEND_MAX_NCOL)))
        nrows = int(math.ceil(len(labels) / ncol))

        # Reserve some top space for the standalone legend.
        top_for_axes = 0.93 - 0.06 * max(0, nrows - 1)
        top_for_axes = max(0.70, min(0.93, top_for_axes))

        fig.legend(
            handles,
            labels,
            fontsize=float(LEGEND_FONTSIZE),
            framealpha=float(LEGEND_FRAMEALPHA),
            loc="upper center",
            bbox_to_anchor=(0.5, 1.0),
            ncol=ncol,
            borderaxespad=0.0,
            columnspacing=0.6,
            handletextpad=0.3,
            labelspacing=0.2,
        )
        fig.tight_layout(pad=0.25, rect=(0.0, 0.0, 1.0, top_for_axes))
    else:
        fig.tight_layout(pad=0.25)

    fig.savefig(out_path, dpi=int(OVERVIEW_DPI), bbox_inches="tight")
    plt.close(fig)


# =========================
# BAG READING / PARSING (only needed topics)
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
        imu_conns = [c for c in conns if _is_type(c.msgtype, T_IMU) and c.topic == "/anymal/imu"]

        wanted = anymal_conns + sea_conns + imu_conns
        if not wanted:
            seen = sorted(set(_norm_type(c.msgtype) for c in conns))
            raise RuntimeError(
                "No supported topics found.\n"
                f"Looking for: {T_ANYMAL_STATE}, {T_SEA_READINGS}, {T_IMU} (/anymal/imu)\n"
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

        joint_keep = "LF_HFE"

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

                    n_num = min(len(pos), len(names) if names else len(pos))
                    if n_num > 0:
                        if len(names) < n_num:
                            names = names + [""] * (n_num - len(names))
                        names = names[:n_num]
                        pos = pos[:n_num]

                        for i in range(n_num):
                            jname = _normalize_joint_name(str(names[i]) if names[i] is not None else "", i)
                            if jname != joint_keep:
                                continue
                            est_data[topic][jname]["t"].append(ns)
                            est_data[topic][jname]["pos"].append(float(pos[i]))

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

                    if jname != joint_keep:
                        continue

                    sea_data[topic][jname]["t"].append(ns)

                    if st is None:
                        sea_data[topic][jname]["pos"].append(float("nan"))
                    else:
                        sea_data[topic][jname]["pos"].append(_safe_float(getattr(st, "joint_position", np.nan)))

                    if cmd is None:
                        sea_data[topic][jname]["cmd_pos"].append(float("nan"))
                    else:
                        sea_data[topic][jname]["cmd_pos"].append(
                            _get_attr_any(
                                cmd,
                                ["position", "joint_position", "joint_position_desired", "desired_position", "pos"],
                                np.nan,
                            )
                        )

            elif _is_type(msgtype, T_IMU):
                av = getattr(msg, "angular_velocity", None)
                la = getattr(msg, "linear_acceleration", None)

                imu_data[topic]["t"].append(ns)
                imu_data[topic]["avx"].append(_safe_float(getattr(av, "x", np.nan)))
                imu_data[topic]["lax"].append(_safe_float(getattr(la, "x", np.nan)))

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

    joint_name = "LF_HFE"
    cmd_pos_topic = _pick_best_cmd_topic_for_joint(sea_data, joint_name, "cmd_pos")

    _plot_overview_vertical(OUTPUT_DIR, t0_ns, est_topic, sea_state_topic, cmd_pos_topic)

    out_file = OUTPUT_DIR / "overview_lf_hfe_contacts_imu_vertical.pdf"
    print(f"Saved: {out_file}")
    print(f"Time window: [{START_TIME_S}, {END_TIME_S if END_TIME_S is not None else 'end'}] s since bag start")
    print(f"AnymalState chosen topic: {est_topic}")
    print(f"SEA state topic:          {sea_state_topic}")
    print(f"SEA cmd_pos topic:        {cmd_pos_topic}")
    print(f"IMU topic:                /anymal/imu  (n={len(imu_data.get('/anymal/imu', {}).get('t', []))})")


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
