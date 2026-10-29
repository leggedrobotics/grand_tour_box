#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import os
import yaml
from box_auto.utils.evaluation_config import validate_evaluation_config

# from evo.core.trajectory import PoseTrajectory3D
from evo.core.trajectory import Plane
from evo.core import sync, lie_algebra
from evo.tools import plot, file_interface
import evo.main_ape as main_ape
import evo.main_rpe as main_rpe
from evo.core.metrics import PoseRelation
from evo.core.metrics import Unit
from evo.tools.settings import SETTINGS
import matplotlib.cm as cm
import matplotlib as mpl
import pandas as pd

# magic plot configuration
import matplotlib.pyplot as plt
import matplotlib.path as mpath
import matplotlib.markers as mmarkers
import matplotlib.collections as mcoll

from matplotlib.patches import Polygon
import matplotlib.patches as patches


# =========================
# GLOBAL FONT + LEGEND CONFIG
# =========================
FONT_FAMILY = "Roboto"  # change if you want, e.g. "Times New Roman"

BASE_FONTSIZE = 16
LABEL_FONTSIZE = 20
TICK_FONTSIZE = 16

# Legend font
LEGEND_FONTSIZE = 20

# Legend width as a fraction of figure width (0.9 == 90%)
LEGEND_WIDTH_FRAC = 0.90

LABEL_WEIGHT = "bold"

# =========================
# Std. Dev. axis unit config
# =========================
STD_DEV_AXIS_UNIT = "mm"  # only affects the y2 axes display
STD_DEV_AXIS_SCALE = 1000.0  # m -> mm

# =========================
# Y-label alignment feature
# =========================
ALIGN_YLABELS = True

# Positions are in "axes coordinates" (0..1) w.r.t. each axis.
# Negative LEFT_YLABEL_X moves the left label further left.
# RIGHT_YLABEL_X > 1 moves the right label further right.
LEFT_YLABEL_X = -0.05
RIGHT_YLABEL_X = 1.05
YLABEL_Y = 0.50  # 0.5 centers vertically

# =========================
# Plot styling parameters
# =========================
STD_DASHED_LINEWIDTH = 1.5  # dashed std-dev line width (y2 axes)
EST_LINEWIDTH_DEFAULT = 1.0  # IE-TC (or default) trajectory line width
# HF trajectory line width will automatically match STD_DASHED_LINEWIDTH

TPS_SCATTER_SIZE = 12  # matplotlib scatter 's' (points^2)
TPS_LEGEND_MARKERSIZE = 15  # legend marker size (points, not area)

STD_CONFIDENCE_SCALE = 50.0  # confidence band scale: y +/- (STD_CONFIDENCE_SCALE * std_dev)

# Legend handle linewidth overrides (requested)
LEGEND_HF_LINEWIDTH = 4.0  # HF line width in legend (handle only)
LEGEND_STD_LINEWIDTH = 4.0  # Std. Dev. dashed line width in legend (handle only)


# =========================
# NEW FEATURE OPTION
# =========================
AUTO_Y_LIMITS = True  # True -> auto y-limits for all plots and both y-axes. False -> keep your manual limits.
AUTO_Y_LIMITS_PADDING_FRAC = 0.15  # same padding factor you used manually


def _finite_min_max(a):
    a = np.asarray(a, dtype=float)
    a = a[np.isfinite(a)]
    if a.size == 0:
        return None, None
    return float(np.min(a)), float(np.max(a))


def _auto_ylim_from_arrays(ax, arrays, pad_frac=AUTO_Y_LIMITS_PADDING_FRAC, clamp_min=None):
    vals = []
    for arr in arrays:
        if arr is None:
            continue
        arr = np.asarray(arr, dtype=float)
        if arr.size == 0:
            continue
        arr = arr[np.isfinite(arr)]
        if arr.size:
            vals.append(arr)

    if not vals:
        return

    data = np.concatenate(vals)
    y_min = float(np.min(data))
    y_max = float(np.max(data))

    if clamp_min is not None:
        y_min = max(y_min, clamp_min)

    # avoid degenerate ranges
    if np.isclose(y_min, y_max):
        span = 1.0 if np.isclose(y_min, 0.0) else abs(y_min) * 0.1
        if span <= 0.0:
            span = 1.0
        y_min -= span
        y_max += span

    pad = pad_frac * (y_max - y_min)
    ax.set_ylim(y_min - pad, y_max + pad)


def _apply_ylabel_alignment(ax_left=None, ax_right=None):
    if not ALIGN_YLABELS:
        return
    if ax_left is not None:
        ax_left.yaxis.set_label_coords(LEFT_YLABEL_X, YLABEL_Y, transform=ax_left.transAxes)
    if ax_right is not None:
        ax_right.yaxis.set_label_coords(RIGHT_YLABEL_X, YLABEL_Y, transform=ax_right.transAxes)


# 1. Build the circle path (unit circle, centered at (0,0))
circle_path = mpath.Path.unit_circle()
circle_verts = circle_path.vertices
circle_codes = circle_path.codes

# 2. Build a small vertical line path from (0, -1) to (0, 1).
line_verts = np.array([[0, -1], [0, 1]])
line_codes = [mpath.Path.MOVETO, mpath.Path.LINETO]

# 3. Combine circle + line into one set of vertices/codes
combined_verts = np.concatenate([circle_verts, line_verts])
combined_codes = np.concatenate([circle_codes, line_codes])

# 4. Create a new Path from that combination
circle_with_vline_path = mpath.Path(combined_verts, combined_codes)

# 5. Make a MarkerStyle from the custom path
custom_marker = mmarkers.MarkerStyle(circle_with_vline_path)


mpl.use("Agg")
mpl.rcParams["agg.path.chunksize"] = 900000000

# Global typography: same font family everywhere
mpl.rcParams.update(
    {
        "font.family": FONT_FAMILY,
        "font.size": BASE_FONTSIZE,
        "axes.labelsize": LABEL_FONTSIZE,
        "axes.labelweight": LABEL_WEIGHT,
        "axes.titlesize": LABEL_FONTSIZE,
        "axes.titleweight": LABEL_WEIGHT,
        "xtick.labelsize": TICK_FONTSIZE,
        "ytick.labelsize": TICK_FONTSIZE,
        "legend.fontsize": LEGEND_FONTSIZE,
        "mathtext.fontset": "dejavusans",
        # embed fonts nicely in vector outputs
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
    }
)

plt.rcParams.update({"figure.max_open_warning": 0})
plt.rcParams.update({"agg.path.chunksize": 9000000})


def run_eval(test_name, reference_file, estimated_file, params, output_path, filter_config, mode):

    # Read the parameters
    t_max_diff = params.get("t_max_diff", 0.02)
    t_offset = params.get("t_offset", 0.0)
    n_to_align = params.get("n_to_align", 800)
    delta = params.get("delta", 1)
    delta_unit = params.get("unit", "m")
    plane_option = filter_config.get("project_to_plane", "xyz")
    if plane_option == "xy":
        plane_param = Plane.XY
    elif plane_option == "xyz":
        plane_param = None
    else:
        print("\033[91mInvalid plane option.\033[0m")
        raise ValueError("Invalid plane option.")

    if delta_unit == "m":
        delta_unit = Unit.meters
    elif delta_unit == "frame":
        delta_unit = Unit.frames
    else:
        print("\033[91mInvalid unit.\033[0m")
        return

    # pick estimated trajectory label from estimated_file name
    est_basename = os.path.basename(estimated_file).lower()
    est_label = "IE-TC" if "ie" in est_basename else "HF"

    # match HF linewidth to dashed std-dev line width
    est_linewidth = STD_DASHED_LINEWIDTH if est_label == "HF" else EST_LINEWIDTH_DEFAULT

    # Figure the name of the test
    if n_to_align == -1:
        alignment_post_fix = "all"
    else:
        alignment_post_fix = str(n_to_align)

    correct_scale = params.get("correct_scale", False)
    test_name += "_point_distances_" + alignment_post_fix

    # Read the trajectories
    traj_reference = file_interface.read_tum_trajectory_file(reference_file)
    traj_estimated = file_interface.read_tum_trajectory_file(estimated_file)

    # Check if the trajectories are valid
    ref_valid, ref_details = traj_reference.check()
    est_valid, est_details = traj_estimated.check()

    if not ref_valid:
        print(
            f"\033[91mReference trajectory is not valid. Not throwing error but warning the user. Check this. : {ref_details}\033[0m"
        )

    if not est_valid:
        print(
            f"\033[91mEstimated trajectory is not valid. Not throwing error but warning the user. Check this. : {est_details}\033[0m"
        )

    # Visually confirm measurement timestamps
    time_diffs = np.diff(traj_estimated.timestamps)
    mean_time_diff = np.mean(time_diffs)
    rmse_time_diff = np.sqrt(np.mean(time_diffs**2))
    std_time_diff = np.std(time_diffs)

    # Check if reference or estimated has more elements
    if len(traj_reference.timestamps) <= len(traj_estimated.timestamps):
        size_of_elements_before_association = len(traj_reference.timestamps)
    else:
        size_of_elements_before_association = len(traj_estimated.timestamps)

    try:
        # Actual association
        traj_reference, traj_estimated = sync.associate_trajectories(
            traj_reference, traj_estimated, t_max_diff, t_offset
        )
    except Exception as e:
        print(f"\033[91mError in association: {e}\033[0m")
        return

    size_of_elements_after_association = len(traj_reference.timestamps)

    if size_of_elements_after_association < 0.5 * size_of_elements_before_association:
        print(
            f"\033[91mNumber of elements in trajectory before association: {size_of_elements_before_association}\033[0m"
        )
        print(
            f"\033[91mNumber of elements in trajectory after association: {size_of_elements_after_association}\033[0m"
        )
        print(
            "\033[91mLess than 10% match. Either time synchronization parameters for association is wrong or a lot of missing AP20 segments.\033[0m"
        )

    # Keep estimates and their uncertainty in their native world frame. Rotate
    # the reference for display, so uncertainty axes do not silently change.
    alignment_transformation = lie_algebra.sim3(
        *traj_reference.align(traj_estimated, correct_scale, False, n=n_to_align)
    )

    traj_reference_original_file = file_interface.read_tum_trajectory_file(reference_file)
    traj_estimated_original_file = file_interface.read_tum_trajectory_file(estimated_file)

    estimated_file_covs_path = estimated_file[:-4] + "_covariance_full.tum"
    std_dev_x, std_dev_y, std_dev_z = get_position_std(
        estimated_file_covs_path, traj_estimated_original_file.timestamps
    ).T
    origin_time = traj_reference_original_file.timestamps[0]
    traj_estimated_original_file.timestamps -= origin_time
    traj_reference_original_file.timestamps -= origin_time

    # scale for y2 axes display only (keep std_dev_* in meters for the uncertainty band math)
    std_dev_x_disp = std_dev_x * STD_DEV_AXIS_SCALE
    std_dev_y_disp = std_dev_y * STD_DEV_AXIS_SCALE
    std_dev_z_disp = std_dev_z * STD_DEV_AXIS_SCALE

    traj_reference_original_file.transform(alignment_transformation)

    fig, axs = plt.subplots(3, 1, figsize=(12, 12))
    fig.suptitle("Raw trajectories in estimator world frame (reference aligned; evaluation filters not applied)")

    gigachad_colors = [
        (0.368, 0.107, 0.71),
        (0.881, 0.611, 0.142),
        (0.923, 0.386, 0.209),
        (0.56, 0.692, 0.195),
        (0.528, 0.471, 0.701),
        (0.772, 0.432, 0.102),
        (0.572, 0.586, 0.0),
    ]
    gmsf = gigachad_colors[0]
    orange = gigachad_colors[1]
    tps = gigachad_colors[3]
    fill_alpha = 0.3

    band_label = f"{STD_CONFIDENCE_SCALE:g}x Std. Dev."

    # ======================
    # X subplot (axs[0])
    # ======================
    ax2 = axs[0].twinx()
    axs[0].set_zorder(2)
    ax2.set_zorder(2)
    ax2.plot(
        traj_estimated_original_file.timestamps,
        std_dev_x_disp,
        color=orange,
        label="Std. Dev.",
        zorder=4,
        linestyle="--",
        linewidth=STD_DASHED_LINEWIDTH,
    )
    ax2.set_ylabel(f"Std. Dev. [{STD_DEV_AXIS_UNIT}]", color="black")

    yticks_1 = [-20, -10, 0, 10, 20]
    yticks_2 = [0, 20, 40, 60]  # in mm
    y1_min = yticks_1[0] - 0.15 * (yticks_1[-1] - yticks_1[0])
    y1_max = yticks_1[-1] + 0.15 * (yticks_1[-1] - yticks_1[0])
    y2_min = yticks_2[0] - 0.15 * (yticks_2[-1] - yticks_2[0])
    y2_max = yticks_2[-1] + 0.15 * (yticks_2[-1] - yticks_2[0])
    if not AUTO_Y_LIMITS:
        axs[0].set_yticks(yticks_1)
        axs[0].set_ylim(y1_min, y1_max)
        ax2.set_yticks(yticks_2)
        ax2.set_ylim(y2_min, y2_max)

    axs[0].plot(
        traj_estimated_original_file.timestamps,
        traj_estimated_original_file.positions_xyz[:, 0],
        c=gmsf,
        label=est_label,
        linewidth=est_linewidth,
        zorder=2,
    )

    axs[0].scatter(
        traj_reference_original_file.timestamps,
        traj_reference_original_file.positions_xyz[:, 0],
        marker="o",
        color=tps,
        label="TPS Measurements",
        s=TPS_SCATTER_SIZE,
        zorder=1,
        rasterized=True,
    )

    axs[0].fill_between(
        traj_estimated_original_file.timestamps,
        traj_estimated_original_file.positions_xyz[:, 0] - (std_dev_x * STD_CONFIDENCE_SCALE),
        traj_estimated_original_file.positions_xyz[:, 0] + (std_dev_x * STD_CONFIDENCE_SCALE),
        color=orange,
        alpha=fill_alpha,
        label=band_label,
        zorder=0,
    )

    if AUTO_Y_LIMITS:
        x_est = traj_estimated_original_file.positions_xyz[:, 0]
        x_ref = traj_reference_original_file.positions_xyz[:, 0]
        x_low = x_est - (std_dev_x * STD_CONFIDENCE_SCALE)
        x_high = x_est + (std_dev_x * STD_CONFIDENCE_SCALE)
        _auto_ylim_from_arrays(axs[0], [x_est, x_ref, x_low, x_high], pad_frac=AUTO_Y_LIMITS_PADDING_FRAC)
        _auto_ylim_from_arrays(
            ax2, [std_dev_x_disp, np.array([0.0])], pad_frac=AUTO_Y_LIMITS_PADDING_FRAC, clamp_min=0.0
        )

    axs[0].set_xlabel("Time [s]")
    axs[0].set_ylabel("X [m]")
    _apply_ylabel_alignment(ax_left=axs[0], ax_right=ax2)

    axs[0].tick_params(axis="y", which="major")
    axs[0].tick_params(axis="x", which="major")
    ax2.tick_params(axis="y", which="major")

    axs[0].grid(which="minor", color="r", linestyle="-", alpha=0.2, zorder=0)

    # ======================
    # Y subplot (axs[1])
    # ======================
    ax1_y2 = axs[1].twinx()
    axs[1].set_zorder(2)
    ax1_y2.set_zorder(2)
    ax1_y2.patch.set_visible(False)
    ax1_y2.plot(
        traj_estimated_original_file.timestamps,
        std_dev_y_disp,
        linestyle="--",
        color=orange,
        label="Std. Dev.",
        linewidth=STD_DASHED_LINEWIDTH,
    )
    ax1_y2.set_ylabel(f"Std. Dev. [{STD_DEV_AXIS_UNIT}]", color="black")

    yticks_1 = [-30, -15, 0, 15, 30]
    yticks_2 = [0, 20, 40, 60]  # in mm
    y1_min = yticks_1[0] - 0.15 * (yticks_1[-1] - yticks_1[0])
    y1_max = yticks_1[-1] + 0.15 * (yticks_1[-1] - yticks_1[0])
    y2_min = yticks_2[0] - 0.15 * (yticks_2[-1] - yticks_2[0])
    y2_max = yticks_2[-1] + 0.15 * (yticks_2[-1] - yticks_2[0])
    if not AUTO_Y_LIMITS:
        axs[1].set_yticks(yticks_1)
        axs[1].set_ylim(y1_min, y1_max)
        ax1_y2.set_yticks(yticks_2)
        ax1_y2.set_ylim(y2_min, y2_max)

    axs[1].plot(
        traj_estimated_original_file.timestamps,
        traj_estimated_original_file.positions_xyz[:, 1],
        c=gmsf,
        label=est_label,
        linewidth=est_linewidth,
        zorder=2,
    )

    axs[1].scatter(
        traj_reference_original_file.timestamps,
        traj_reference_original_file.positions_xyz[:, 1],
        edgecolor="none",
        color=tps,
        label="TPS Measurements",
        s=TPS_SCATTER_SIZE,
        zorder=1,
        rasterized=True,
    )

    axs[1].fill_between(
        traj_estimated_original_file.timestamps,
        traj_estimated_original_file.positions_xyz[:, 1] - (std_dev_y * STD_CONFIDENCE_SCALE),
        traj_estimated_original_file.positions_xyz[:, 1] + (std_dev_y * STD_CONFIDENCE_SCALE),
        color=orange,
        alpha=fill_alpha,
        label=band_label,
        zorder=0,
    )

    if AUTO_Y_LIMITS:
        y_est = traj_estimated_original_file.positions_xyz[:, 1]
        y_ref = traj_reference_original_file.positions_xyz[:, 1]
        y_low = y_est - (std_dev_y * STD_CONFIDENCE_SCALE)
        y_high = y_est + (std_dev_y * STD_CONFIDENCE_SCALE)
        _auto_ylim_from_arrays(axs[1], [y_est, y_ref, y_low, y_high], pad_frac=AUTO_Y_LIMITS_PADDING_FRAC)
        _auto_ylim_from_arrays(
            ax1_y2, [std_dev_y_disp, np.array([0.0])], pad_frac=AUTO_Y_LIMITS_PADDING_FRAC, clamp_min=0.0
        )

    axs[1].set_xlabel("Time [s]")
    axs[1].set_ylabel("Y [m]")
    _apply_ylabel_alignment(ax_left=axs[1], ax_right=ax1_y2)

    axs[1].tick_params(axis="y", which="major")
    axs[1].tick_params(axis="x", which="major")
    ax1_y2.tick_params(axis="y", which="major")

    axs[1].grid(True)

    # ======================
    # Z subplot (axs[2])
    # ======================
    ax2_y2 = axs[2].twinx()
    axs[2].set_zorder(2)
    ax2_y2.set_zorder(2)
    ax2_y2.patch.set_visible(False)
    ax2_y2.plot(
        traj_estimated_original_file.timestamps,
        std_dev_z_disp,
        color=orange,
        label="Std. Dev.",
        linestyle="--",
        linewidth=STD_DASHED_LINEWIDTH,
    )
    ax2_y2.set_ylabel(f"Std. Dev. [{STD_DEV_AXIS_UNIT}]", color="black")

    yticks_1 = [-2, -1, 0, 1, 2]
    yticks_2 = [0, 20, 40, 60]  # in mm
    y1_min = yticks_1[0] - 0.15 * (yticks_1[-1] - yticks_1[0])
    y1_max = yticks_1[-1] + 0.15 * (yticks_1[-1] - yticks_1[0])
    y2_min = yticks_2[0] - 0.15 * (yticks_2[-1] - yticks_2[0])
    y2_max = yticks_2[-1] + 0.15 * (yticks_2[-1] - yticks_2[0])
    if not AUTO_Y_LIMITS:
        axs[2].set_yticks(yticks_1)
        axs[2].set_ylim(y1_min, y1_max)
        ax2_y2.set_yticks(yticks_2)
        ax2_y2.set_ylim(y2_min, y2_max)

    axs[2].tick_params(axis="y", which="major")
    axs[2].tick_params(axis="x", which="major")
    ax2_y2.tick_params(axis="y", which="major")

    axs[2].plot(
        traj_estimated_original_file.timestamps,
        traj_estimated_original_file.positions_xyz[:, 2],
        c=gmsf,
        label=est_label,
        linewidth=est_linewidth,
        zorder=2,
    )

    axs[2].scatter(
        traj_reference_original_file.timestamps,
        traj_reference_original_file.positions_xyz[:, 2],
        marker="o",
        color=tps,
        label="TPS Measurements",
        s=TPS_SCATTER_SIZE,
        zorder=1,
        rasterized=True,
    )

    axs[2].fill_between(
        traj_estimated_original_file.timestamps,
        traj_estimated_original_file.positions_xyz[:, 2] - (std_dev_z * STD_CONFIDENCE_SCALE),
        traj_estimated_original_file.positions_xyz[:, 2] + (std_dev_z * STD_CONFIDENCE_SCALE),
        color=orange,
        alpha=fill_alpha,
        label=band_label,
        zorder=0,
    )

    if AUTO_Y_LIMITS:
        z_est = traj_estimated_original_file.positions_xyz[:, 2]
        z_ref = traj_reference_original_file.positions_xyz[:, 2]
        z_low = z_est - (std_dev_z * STD_CONFIDENCE_SCALE)
        z_high = z_est + (std_dev_z * STD_CONFIDENCE_SCALE)
        _auto_ylim_from_arrays(axs[2], [z_est, z_ref, z_low, z_high], pad_frac=AUTO_Y_LIMITS_PADDING_FRAC)
        _auto_ylim_from_arrays(
            ax2_y2, [std_dev_z_disp, np.array([0.0])], pad_frac=AUTO_Y_LIMITS_PADDING_FRAC, clamp_min=0.0
        )

    axs[2].set_xlabel("Time [s]")
    axs[2].set_ylabel("Z [m]")
    _apply_ylabel_alignment(ax_left=axs[2], ax_right=ax2_y2)

    # ======================
    # Single figure legend (top, width = LEGEND_WIDTH_FRAC)
    # ======================
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch

    # legend handle linewidth overrides apply only to HF + Std. Dev.
    legend_est_lw = LEGEND_HF_LINEWIDTH if est_label == "HF" else est_linewidth

    legend_handles = [
        Line2D([0], [0], color=gmsf, lw=legend_est_lw),
        Line2D([0], [0], marker="o", linestyle="None", color=tps, markersize=TPS_LEGEND_MARKERSIZE),
        Patch(facecolor=orange, edgecolor=orange, alpha=fill_alpha),
        Line2D([0], [0], color=orange, lw=LEGEND_STD_LINEWIDTH, linestyle="--"),
    ]
    legend_labels = [
        est_label,
        "TPS Measurements",
        band_label,
        "Std. Dev.",
    ]

    # Legend band placement at top
    legend_y0 = 0.92
    legend_h = 0.08

    # Center a bbox of width LEGEND_WIDTH_FRAC in figure coordinates
    legend_x0 = (1.0 - LEGEND_WIDTH_FRAC) * 0.5

    fig_leg = fig.legend(
        legend_handles,
        legend_labels,
        loc="lower left",
        bbox_to_anchor=(legend_x0, legend_y0, LEGEND_WIDTH_FRAC, legend_h),
        bbox_transform=fig.transFigure,
        mode="expand",
        ncol=len(legend_labels),
        framealpha=1.0,
        edgecolor="black",
        fancybox=False,
        borderaxespad=0.0,
        handlelength=2.0,
        columnspacing=1.2,
        prop={"size": LEGEND_FONTSIZE, "family": FONT_FAMILY},
    )
    fig_leg.set_zorder(10000)
    fig_leg.get_frame().set_linewidth(2.0)
    fig_leg.get_frame().set_facecolor("1.0")

    # ======================
    # Styling + save
    # ======================
    axs[0].grid(visible=True, color="black", linestyle="--", linewidth=0.5)
    axs[1].grid(visible=True, color="black", linestyle="--", linewidth=0.5)
    axs[2].grid(visible=True, color="black", linestyle="--", linewidth=0.5)
    ax2.grid(visible=False)
    ax1_y2.grid(visible=False)
    ax2_y2.grid(visible=False)

    axs[0].set_facecolor((245 / 255, 245 / 255, 245 / 255))
    axs[1].set_facecolor((245 / 255, 245 / 255, 245 / 255))
    axs[2].set_facecolor((245 / 255, 245 / 255, 245 / 255))

    # leave space at top for the legend band
    plt.tight_layout(rect=[0.0, 0.0, 1.0, legend_y0 - 0.02])
    plt.savefig(output_path + "/gmsf_scatter_plot_XYZ_" + test_name + ".pdf", dpi=600)
    plt.close(fig)


def get_position_std(path, timestamps):
    """Read full pose covariance exported for exactly these transformed poses."""
    data = np.loadtxt(path, skiprows=1, ndmin=2)
    if data.shape != (len(timestamps), 37) or not np.isfinite(data).all():
        raise ValueError("Full covariance rows must match the pose trajectory; re-export with evo_conversion_node")
    if not np.allclose(data[:, 0], timestamps, rtol=0, atol=1e-6):
        raise ValueError("Covariance timestamps do not match pose timestamps")
    covariance = data[:, 1:].reshape(-1, 6, 6)
    if (
        not np.allclose(covariance, covariance.transpose(0, 2, 1), atol=1e-10)
        or np.linalg.eigvalsh(covariance).min() < -1e-10
    ):
        raise ValueError("Invalid pose covariance")
    return np.sqrt(np.maximum(np.diagonal(covariance[:, :3, :3], axis1=1, axis2=2), 0))


def get_covariances(reference_file_covs_path):
    data = pd.read_csv(
        reference_file_covs_path,
        sep=" ",
        header=0,
        names=["timestamp", "x", "y", "z", "roll", "pitch", "yaw"],
    )

    x_cov = np.array(data.x)
    y_cov = np.array(data.y)
    z_cov = np.array(data.z)

    roll_cov = np.array(data.roll)
    pitch_cov = np.array(data.pitch)
    yaw_cov = np.array(data.yaw)

    return x_cov, y_cov, z_cov, roll_cov, pitch_cov, yaw_cov


def get_top_n_percentile_indices(covariances, n):
    if len(covariances) == 0:
        return []
    threshold = np.percentile(covariances, n)
    return np.where(covariances > threshold)[0]


def process_directory(base_path, output_dir, config, disable_viz, prefix):

    # This diagnostic intentionally displays all raw samples. Metric filtering
    # remains in grandtour_point_relation.py and is not duplicated here.
    filter_config = {"project_to_plane": config.get("project_to_plane", "xyz")}

    for param_set in config["bag_topic_pairs"]:
        if param_set.get("test_name") == "":
            continue

        test_name = param_set.get("test_name")
        eval_params = param_set["evaluations"]
        ref_file = eval_params.get("point_relation")["reference_file"]

        if not ref_file:
            continue

        reference_file = os.path.join(base_path, ref_file + ".tum")
        estimated_file = test_name + ".tum"
        if estimated_file.startswith("_"):
            estimated_file = prefix + estimated_file

        estimated_file = os.path.join(base_path, estimated_file)
        params = eval_params.get("parameters")

        print(f"\033[92mProcessing test: {test_name}\033[0m")
        print(f"\033[92mEstimated file: {estimated_file}, Reference file: {reference_file}\033[0m")

        if "ap20" in test_name:
            print(f"\033[93mSkipping test: {test_name} due to 'ap20' in the name.\033[0m")
            continue

        run_eval(test_name, reference_file, estimated_file, params, output_dir, filter_config, "ate")
        # run_eval(test_name, reference_file, estimated_file, params, output_dir, filter_config, "rte")


def load_config(config_path):
    with open(config_path, "r") as f:
        config = validate_evaluation_config(yaml.safe_load(f))
    return config


def main(argv=None):
    import argparse
    from pathlib import Path

    parser = argparse.ArgumentParser(description="Plot raw trajectories and full covariance in the estimator frame")
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--prefix", default="")
    args = parser.parse_args(argv)
    if args.output.exists():
        raise FileExistsError("Use a fresh plot output directory: " + str(args.output))
    config = load_config(args.config)
    args.output.mkdir(parents=True)
    process_directory(str(args.input), str(args.output), config, False, args.prefix)


if __name__ == "__main__":
    main()
