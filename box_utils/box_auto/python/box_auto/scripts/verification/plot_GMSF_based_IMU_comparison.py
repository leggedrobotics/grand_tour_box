#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import os
import yaml

# from evo.core.trajectory import PoseTrajectory3D
from evo.core import sync, lie_algebra
from evo.tools import file_interface
from evo.core.metrics import Unit
import matplotlib as mpl
import pandas as pd

# SETTINGS.plot_figsize = [12, 8]
# SETTINGS.plot_split = True
# SETTINGS.plot_usetex = False
# SETTINGS.plot_xyz_realistic = False
# SETTINGS.plot_show_legend = True
# SETTINGS.plot_show_axis = True

# magic plot configuration
import matplotlib.pyplot as plt
import matplotlib.path as mpath
import matplotlib.markers as mmarkers
import matplotlib.collections as mcoll

# SETTINGS.plot_figsize = [12, 8]
# SETTINGS.plot_split = True
# SETTINGS.plot_usetex = False
# SETTINGS.plot_xyz_realistic = False
# SETTINGS.plot_show_legend = True
# SETTINGS.plot_show_axis = True

# 1. Build the circle path (unit circle, centered at (0,0))
circle_path = mpath.Path.unit_circle()
circle_verts = circle_path.vertices
circle_codes = circle_path.codes

# 2. Build a small vertical line path from (0, -1) to (0, 1).
#    You can shrink or stretch these coordinates if you want
#    the line to be shorter or longer relative to the circle.
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
plt.rcParams.update({"figure.max_open_warning": 0})
plt.rcParams.update({"agg.path.chunksize": 9000000})

# mpl.rcParams['patch.antialiased'] = False


def run_eval(test_name, reference_file, estimated_file, params, output_path, filter_config, mode):

    # Read the parameters
    t_max_diff = params.get("t_max_diff", 0.02)
    t_offset = params.get("t_offset", 0.0)
    n_to_align = params.get("n_to_align", 800)
    # delta = params.get("delta", 1)
    delta_unit = params.get("unit", "m")
    # plane_option = filter_config.get("project_to_plane", "xyz")
    # if plane_option == "xy":
    #     plane_param = Plane.XY
    # elif plane_option == "xyz":
    #     plane_param = None
    # else:
    #     print("\033[91mInvalid plane option.\033[0m")
    #     raise ValueError("Invalid plane option.")

    if delta_unit == "m":
        delta_unit = Unit.meters
    elif delta_unit == "frame":
        delta_unit = Unit.frames
    else:
        print("\033[91mInvalid unit.\033[0m")
        return

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

    # Check if the trajectories are valid, first thing to do.
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
    # time_diffs = []
    # time_diffs = np.diff(traj_estimated.timestamps)
    # mean_time_diff = np.mean(time_diffs)
    # rmse_time_diff = np.sqrt(np.mean(time_diffs**2))
    # std_time_diff = np.std(time_diffs)

    # plt.figure(figsize=(10, 6))
    # # plt.plot(time_diffs, label="Time Differences")
    # plt.plot(np.where(time_diffs < 1, time_diffs, np.nan), label="Time Differences < 1s")
    # plt.xlabel("Index")
    # plt.ylabel("Time Difference (s)")
    # plt.title("Time Differences Between Sequential Entries")
    # plt.legend()
    # plt.grid(True)

    # textstr = "\n".join(
    #     (
    #         f"Mean: {mean_time_diff:.4f} s",
    #         f"RMSE: {rmse_time_diff:.4f} s",
    #         f"STD: {std_time_diff:.4f} s",
    #     )
    # )
    # plt.gcf().text(0.5, 0.90, textstr, fontsize=12, ha="center")
    # plt.savefig(output_path + "/time_differences_before_" + test_name + ".png", dpi=300)
    # plt.close()

    # Filter the trajectories
    # if filter_config.get("enable_covariance_based_removal", False):
    #     # Read the covariances, can be a empty file.
    #     estimated_file_covs_path = estimated_file[:-4] + "_covariance.tum"
    #     est_x_cov, est_y_cov, est_z_cov, est_roll_cov, est_pitch_cov, est_yaw_cov = get_covariances(
    #         estimated_file_covs_path
    #     )

    #     if len(est_x_cov) != 0 and len(est_x_cov) != len(traj_estimated.positions_xyz):
    #         raise ValueError("Size of est_x_cov is not empty and not the same size as traj_estimated.")

    #     threshold = filter_config.get("covariance_percentile_threshold", 95)

    #     est_x_cov_indices_to_remove = get_top_n_percentile_indices(est_x_cov, threshold)
    #     est_y_cov_indices_to_remove = get_top_n_percentile_indices(est_y_cov, threshold)
    #     est_z_cov_indices_to_remove = get_top_n_percentile_indices(est_z_cov, threshold)
    #     est_roll_cov_indices_to_remove = get_top_n_percentile_indices(est_roll_cov, threshold)
    #     est_pitch_cov_indices_to_remove = get_top_n_percentile_indices(est_pitch_cov, threshold)
    #     est_yaw_cov_indices_to_remove = get_top_n_percentile_indices(est_yaw_cov, threshold)
    #     all_indices_to_remove = np.unique(
    #         np.concatenate(
    #             [
    #                 est_x_cov_indices_to_remove,
    #                 est_y_cov_indices_to_remove,
    #                 est_z_cov_indices_to_remove,
    #                 est_roll_cov_indices_to_remove,
    #                 est_pitch_cov_indices_to_remove,
    #                 est_yaw_cov_indices_to_remove,
    #             ]
    #         )
    #     )

    #     indices_to_keep = np.setdiff1d(np.arange(len(traj_estimated.positions_xyz)), all_indices_to_remove)
    #     traj_estimated.reduce_to_ids(indices_to_keep)

    # if filter_config.get("enable_no_motion_removal", False):
    #     # Find indices where the difference to the first value of traj_estimated.positions_xyz is more than 50cm
    #     distances = np.linalg.norm(traj_estimated.positions_xyz - traj_estimated.positions_xyz[0], axis=1)
    #     indices_to_keep = np.where(distances > filter_config.get("distance_threshold", 10))[0]
    #     traj_estimated.reduce_to_ids(indices_to_keep)

    # if filter_config.get("ap20_peak_rejection", False):
    #     # Remove the measurements around the lost AP20 measurements
    #     trailing_window = filter_config.get("ap20_peak_rejection_trailing_window", 5)
    #     time_diffs = []
    #     for i in range(1, len(traj_reference.timestamps)):
    #         time_diffs.append(traj_reference.timestamps[i] - traj_reference.timestamps[i - 1])
    #     time_diffs = np.array(time_diffs)

    #     indices_to_remove = np.where(time_diffs > filter_config.get("ap20_peak_rejection_threshold", 0.1))[0]
    #     extended_indices_to_remove = set(indices_to_remove)
    #     for idx in indices_to_remove:
    #         extended_indices_to_remove.update(range(idx, idx + trailing_window))

    #         if idx - trailing_window >= 0:
    #             extended_indices_to_remove.update(range(idx - trailing_window, idx))

    #     extended_indices_to_remove = np.unique(np.array(sorted(extended_indices_to_remove)))

    #     # Get the indices to keep
    #     indices_to_keep = np.setdiff1d(np.arange(len(traj_reference.positions_xyz)), extended_indices_to_remove)

    #     # Reduce the trajectory to the indices to keep
    #     traj_reference.reduce_to_ids(indices_to_keep)

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

    # Both will have the same size so doesnt matter.
    size_of_elements_after_association = len(traj_reference.timestamps)

    # Warn user about the number of elements in the trajectory after association
    if size_of_elements_after_association < 0.5 * size_of_elements_before_association:
        print(
            f"\033[91mNumber of elements in trajectory before association: {size_of_elements_before_association}\033[0m"
        )
        print(
            f"\033[91mNumber of elements in trajectory after association: {size_of_elements_after_association}\033[0m"
        )
        print(
            "\033[91mLess than 50% match. Either time synchronization parameters for association is wrong or a lot of missing AP20 segments.\033[0m"
        )

    alignment_transformation = lie_algebra.sim3(
        *traj_estimated.align(traj_reference, correct_scale, False, n=n_to_align)
    )

    # results = []
    # fig_title = ""
    # if mode == "ate":
    #     save_prefix = "ate"
    #     fig_title = "Absolute Translation Error (ATE) Trajectory"
    #     result = main_ape.ape(
    #         traj_reference,
    #         traj_estimated,
    #         est_name=test_name,
    #         ref_name="AP20",
    #         pose_relation=PoseRelation.point_distance,
    #         align=True,
    #         align_origin=False,
    #         n_to_align=n_to_align,
    #         correct_scale=correct_scale,
    #         project_to_plane=plane_param,
    #     )
    # elif mode == "rte":
    #     save_prefix = "rte"
    #     fig_title = "Relative Translation Error (RTE) Trajectory"
    #     result = main_rpe.rpe(
    #         traj_reference,
    #         traj_estimated,
    #         est_name=test_name,
    #         ref_name="AP20",
    #         pose_relation=PoseRelation.point_distance,
    #         delta=delta,
    #         delta_unit=delta_unit,
    #         all_pairs=False,
    #         align=True,
    #         correct_scale=correct_scale,
    #         n_to_align=n_to_align,
    #         project_to_plane=plane_param,
    #         support_loop=False,
    #     )

    # results.append(result)

    traj_reference_original_file = file_interface.read_tum_trajectory_file(reference_file)
    traj_estimated_original_file = file_interface.read_tum_trajectory_file(estimated_file)

    traj_estimated_original_file.timestamps = (
        traj_estimated_original_file.timestamps - traj_reference_original_file.timestamps[0]
    )
    traj_reference_original_file.timestamps = (
        traj_reference_original_file.timestamps - traj_reference_original_file.timestamps[0]
    )

    # invalid_indices = np.where(
    #     (traj_estimated_original_file.timestamps < 0) |
    #     (traj_estimated_original_file.timestamps > traj_reference_original_file.timestamps[-1])
    # )[0]

    # # Get the indices to keep
    # indices_to_keep = np.setdiff1d(np.arange(len(traj_estimated_original_file.positions_xyz)), invalid_indices)

    # Plot and save traj_reference.positions_xyz as scatter plot
    estimated_file_covs_path = estimated_file[:-4] + "_covariance.tum"
    std_dev_x, std_dev_y, std_dev_z, _, _, _ = get_covariances(estimated_file_covs_path)

    # std_dev_x = std_dev_x[indices_to_keep]
    # std_dev_y = std_dev_y[indices_to_keep]
    # std_dev_z = std_dev_z[indices_to_keep]

    # Reduce the trajectory to the indices to keep
    # traj_estimated_original_file.reduce_to_ids(indices_to_keep)

    traj_estimated_original_file.transform(alignment_transformation)

    # plt.figure(figsize=(12, 8))
    # plt.scatter(traj_reference.positions_xyz[:, 0], traj_reference.positions_xyz[:, 1], c="blue", label="Reference", s=4)
    # plt.scatter(traj_estimated.positions_xyz[:, 0], traj_estimated.positions_xyz[:, 1], c="red", label="Estimated", s=4)
    # plt.xlabel("X [m]")
    # plt.ylabel("Y [m]")
    # plt.title("Trajectory Scatter Plot")
    # plt.legend()
    # plt.grid(True)
    # plt.savefig(output_path + "/scatter_plot_" + test_name + ".png", dpi=300)
    # plt.close()

    # std_dev_x = std_dev_x * 100
    # std_dev_y = std_dev_y * 100
    # std_dev_z = std_dev_z * 100

    _, axs = plt.subplots(3, 1, figsize=(12, 16))

    # x_start = 84.0
    # x_start2 = 84.0
    # x_end = 250.0

    # Transform coordinates to figure space
    # x_start_fig, _ = axs[0].transData.transform((x_start, 0))
    # x_start_fig = fig.transFigure.inverted().transform((x_start_fig, 0))[0]

    # rect = patches.Rectangle(
    #     (84, -20), 121 - 84, 40, color="orange", alpha=0.3  # (x, y) for lower-left corner  # width  # height
    # )

    # rect_1 = patches.Rectangle(
    #     (84, -20), 121 - 84, 58, color="orange", alpha=0.3, zorder=0  # (x, y) for lower-left corner  # width  # height
    # )

    # rect_2 = patches.Rectangle(
    #     (84, -2), 121 - 84, 5, color="orange", alpha=0.3  # (x, y) for lower-left corner  # width  # height
    # )

    # rect2 = patches.Rectangle((x_start_fig2, 0.135), 0.075, 0.825, transform=fig.transFigure, color='red', alpha=0.2)
    # fig.add_artist(rect2)

    # Define the start and end times for plotting
    A = 0
    B = 100

    # Filter the trajectories to only include timestamps between A and B
    indices_to_keep_estimated = np.where(
        (traj_estimated_original_file.timestamps >= A) & (traj_estimated_original_file.timestamps <= B)
    )[0]
    indices_to_keep_reference = np.where(
        (traj_reference_original_file.timestamps >= A) & (traj_reference_original_file.timestamps <= B)
    )[0]

    traj_estimated_original_file.reduce_to_ids(indices_to_keep_estimated)
    traj_reference_original_file.reduce_to_ids(indices_to_keep_reference)

    std_dev_x = std_dev_x[indices_to_keep_estimated]
    std_dev_y = std_dev_y[indices_to_keep_estimated]
    std_dev_z = std_dev_z[indices_to_keep_estimated]

    ax2 = axs[0].twinx()
    axs[0].set_zorder(2)  # Put ax in front of ax2
    ax2.set_zorder(2)  # Push ax2 behind axs[0]
    # ax2.patch.set_visible(False)  # Hide ax2 background so it doesn't cover axs[0]
    ax2.plot(traj_estimated_original_file.timestamps, std_dev_x, color="orange", label="Std. Dev. X")
    ax2.set_ylabel("Std. Dev. [m]", color="black", fontsize=20, fontweight="bold")
    ax2.set_ylim(0, 0.06)

    # ========== X component scatter + uncertainty ==========
    axs[0].scatter(
        traj_reference_original_file.timestamps,
        traj_reference_original_file.positions_xyz[:, 0],
        marker="*",
        c="blue",
        label="RTS Measurements",
        s=8,
        zorder=1,
    )
    axs[0].scatter(
        traj_estimated_original_file.timestamps,
        traj_estimated_original_file.positions_xyz[:, 0],
        c="red",
        label="GMSF Optimized",
        s=1,
        zorder=1,
    )

    # Fill the region between (estimated - std) and (estimated + std)
    axs[0].fill_between(
        traj_estimated_original_file.timestamps,
        traj_estimated_original_file.positions_xyz[:, 0] - (std_dev_x * 50),
        traj_estimated_original_file.positions_xyz[:, 0] + (std_dev_x * 50),
        color="green",
        alpha=0.3,
        label="50x Std. Dev. X",
        zorder=1,
    )

    axs[0].set_xlabel("Time [s]", fontsize=20, fontweight="bold")
    axs[0].set_ylabel("X [m]", fontsize=20, fontweight="bold")

    axs[0].tick_params(axis="y", which="major", labelsize=16)
    axs[0].tick_params(axis="x", which="major", labelsize=16)
    ax2.tick_params(axis="y", which="major", labelsize=16)

    handles1, labels1 = axs[0].get_legend_handles_labels()  # Primary Y-axis
    handles2, labels2 = ax2.get_legend_handles_labels()  # Twin Y-axis

    # Merge both legends
    combined_handles = handles1 + handles2
    combined_labels = labels1 + labels2

    # Create a single merged legend on axs[0]
    leg = axs[0].legend(
        combined_handles, combined_labels, loc="upper right", fontsize=16, framealpha=1, edgecolor="black"
    )
    leg.set_zorder(3)  # Bring legend to front

    leg.get_frame().set_edgecolor("0.0")
    leg.get_frame().set_facecolor("0.95")
    leg.get_frame().set_linewidth(2.0)
    leg.get_frame().set_alpha(1.0)

    # for legobj in leg.legendHandles:
    #     # legobj.set_linewidth(8.0)
    #     legobj.set_sizes([200])
    #     # legobj.set_markersize(16)

    for handle in leg.legendHandles:
        if isinstance(handle, plt.Line2D):  # If it's a line
            handle.set_linewidth(6.0)
            # handle.set_linestyle("--")
        elif isinstance(handle, mcoll.PathCollection):  # If it's a scatter plot
            handle.set_sizes([200])  # Increase the marker size in legend
            handle.set_alpha(0.6)  # Adjust transparency

    # plt.grid(b=True, which='major', color='k', linestyle='-')
    axs[0].grid(b=True, which="minor", color="r", linestyle="-", alpha=0.2, zorder=0)

    ax1_y2 = axs[1].twinx()
    axs[1].set_zorder(2)  # Put ax in front of ax2
    ax1_y2.set_zorder(2)  # Push ax2 behind axs[0]
    ax1_y2.patch.set_visible(False)  # Hide ax2 background so it doesn't cover axs[0]
    ax1_y2.plot(traj_estimated_original_file.timestamps, std_dev_y, color="orange", label="Std. Dev. Y")
    ax1_y2.set_ylabel("Std. Dev. [m]", color="black", fontsize=20, fontweight="bold")
    ax1_y2.set_ylim(0, 0.06)

    # ========== Y component scatter + uncertainty ==========
    axs[1].scatter(
        traj_reference_original_file.timestamps,
        traj_reference_original_file.positions_xyz[:, 1],
        marker="d",
        c="blue",
        label="RTS Measurements",
        s=8,
        zorder=1,
    )
    axs[1].scatter(
        traj_estimated_original_file.timestamps,
        traj_estimated_original_file.positions_xyz[:, 1],
        c="red",
        label="GMSF Optimized",
        s=1,
        zorder=1,
    )

    axs[1].fill_between(
        traj_estimated_original_file.timestamps,
        traj_estimated_original_file.positions_xyz[:, 1] - (std_dev_y * 50),
        traj_estimated_original_file.positions_xyz[:, 1] + (std_dev_y * 50),
        color="green",
        alpha=0.3,
        label="50x Std. Dev. Y",
        zorder=1,
    )

    axs[1].set_xlabel("Time [s]", fontsize=20, fontweight="bold")
    axs[1].set_ylabel("Y [m]", fontsize=20, fontweight="bold")

    axs[1].tick_params(axis="y", which="major", labelsize=16)
    axs[1].tick_params(axis="x", which="major", labelsize=16)
    ax1_y2.tick_params(axis="y", which="major", labelsize=16)

    handles1, labels1 = axs[1].get_legend_handles_labels()  # Primary Y-axis
    handles2, labels2 = ax1_y2.get_legend_handles_labels()  # Twin Y-axis

    # Merge both legends
    combined_handles = handles1 + handles2
    combined_labels = labels1 + labels2

    # Create a single merged legend on axs[0]
    leg = axs[1].legend(
        combined_handles, combined_labels, loc="upper right", fontsize=16, framealpha=1, edgecolor="black"
    )
    leg.set_zorder(3)  # Bring legend to front

    leg.get_frame().set_edgecolor("0.0")
    leg.get_frame().set_facecolor("0.95")
    leg.get_frame().set_linewidth(2.0)
    leg.get_frame().set_alpha(1.0)
    axs[1].grid(True)

    for handle in leg.legendHandles:
        if isinstance(handle, plt.Line2D):  # If it's a line
            handle.set_linewidth(6.0)
            # handle.set_linestyle("--")
        elif isinstance(handle, mcoll.PathCollection):  # If it's a scatter plot
            handle.set_sizes([200])  # Increase the marker size in legend
            handle.set_alpha(0.6)  # Adjust transparency

    ax2_y2 = axs[2].twinx()
    axs[2].set_zorder(2)  # Put ax in front of ax2
    ax2_y2.set_zorder(2)  # Push ax2 behind axs[0]
    ax2_y2.patch.set_visible(False)  # Hide ax2 background so it doesn't cover axs[0]
    ax2_y2.plot(traj_estimated_original_file.timestamps, std_dev_z, color="orange", label="Std. Dev. Z")
    ax2_y2.set_ylabel("Std. Dev. [m]", color="black", fontsize=20, fontweight="bold")
    ax2_y2.set_ylim(0, 0.06)

    axs[2].tick_params(axis="y", which="major", labelsize=16)
    axs[2].tick_params(axis="x", which="major", labelsize=16)
    ax2_y2.tick_params(axis="y", which="major", labelsize=16)

    # ========== Z component scatter + uncertainty ==========
    axs[2].scatter(
        traj_reference_original_file.timestamps,
        traj_reference_original_file.positions_xyz[:, 2],
        marker="o",
        c="blue",
        label="RTS Measurements",
        s=8,
    )
    axs[2].scatter(
        traj_estimated_original_file.timestamps,
        traj_estimated_original_file.positions_xyz[:, 2],
        c="red",
        label="GMSF Optimized",
        s=1,
    )

    axs[2].fill_between(
        traj_estimated_original_file.timestamps,
        traj_estimated_original_file.positions_xyz[:, 2] - (std_dev_z * 50),
        traj_estimated_original_file.positions_xyz[:, 2] + (std_dev_z * 50),
        color="green",
        alpha=0.3,
        label="50x Std. Dev. Z",
    )

    axs[2].set_xlabel("Time [s]", fontsize=20, fontweight="bold")
    axs[2].set_ylabel("Z [m]", fontsize=20, fontweight="bold")

    handles1, labels1 = axs[2].get_legend_handles_labels()  # Primary Y-axis
    handles2, labels2 = ax1_y2.get_legend_handles_labels()  # Twin Y-axis

    # Merge both legends
    combined_handles = handles1 + handles2
    combined_labels = labels1 + labels2

    leg = axs[2].legend(
        combined_handles, combined_labels, loc="upper right", fontsize=16, framealpha=1, edgecolor="black"
    )

    leg.set_zorder(3)  # Bring legend to front

    leg.get_frame().set_edgecolor("0.0")
    leg.get_frame().set_facecolor("0.95")
    leg.get_frame().set_linewidth(2.0)
    leg.get_frame().set_alpha(1.0)

    for handle in leg.legendHandles:
        if isinstance(handle, plt.Line2D):  # If it's a line
            handle.set_linewidth(6.0)
            # handle.set_linestyle("--")
        elif isinstance(handle, mcoll.PathCollection):  # If it's a scatter plot
            handle.set_sizes([200])  # Increase the marker size in legend
            handle.set_alpha(0.6)  # Adjust transparency

    # axs[2].grid(True)
    axs[0].grid(visible=True, color="black", linestyle="--", linewidth=0.5)
    axs[1].grid(visible=True, color="black", linestyle="--", linewidth=0.5)
    axs[2].grid(visible=True, color="black", linestyle="--", linewidth=0.5)
    ax2.grid(visible=False)
    ax1_y2.grid(visible=False)
    ax2_y2.grid(visible=False)
    # axs[0].set_xlim(0, 100)
    # axs[1].set_xlim(0, 100)
    # axs[2].set_xlim(250, 300)

    # axs[0].set_ylim(-20, 0)
    # axs[1].set_ylim(0, 25)
    # axs[2].set_ylim(-12, -5)

    # plt.tight_layout()
    # plt.show()
    plt.savefig(output_path + "/IMUs_scatter_plot_XYZ_" + test_name + ".pdf", dpi=600)


def get_covariances(reference_file_covs_path):
    data = pd.read_csv(
        reference_file_covs_path, sep=" ", header=0, names=["timestamp", "x", "y", "z", "roll", "pitch", "yaw"]
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

    filter_config = {
        "enable_covariance_based_removal": config["filter_by_covariance"],
        "covariance_percentile_threshold": config["covariance_percentile_threshold"],
        "enable_no_motion_removal": config["remove_until_motion"],
        "distance_threshold": config["distance_threshold"],
        "ap20_peak_rejection": config["ap20_peak_rejection"],
        "ap20_peak_rejection_threshold": config["ap20_peak_rejection_threshold"],
        "ap20_peak_rejection_trailing_window": config["ap20_peak_rejection_trailing_window"],
        "project_to_plane": config["project_to_plane"],
        "disable_viz": disable_viz,
    }

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

        if "ap20_robot" in test_name:
            print(f"\033[93mSkipping test: {test_name} due to 'ap20' in the name.\033[0m")
            continue
        run_eval(test_name, reference_file, estimated_file, params, output_dir, filter_config, "ate")
        # run_eval(test_name, reference_file, estimated_file, params, output_dir, filter_config, "rte")


def load_config(config_path):
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
    return config


if __name__ == "__main__":

    # import argparse

    # parser = argparse.ArgumentParser(description="Run ATE and RTE evaluation for multiple tests.")

    # # Define command-line arguments
    # parser.add_argument("--config", type=str, required=True, help="Path to the configuration YAML file")
    # parser.add_argument(
    #     "--input_folder_path",
    #     type=str,
    #     required=True,
    #     default="mission_data/",
    #     help="Mission data subdirectory (default: 'mission_data/')",
    # )
    # parser.add_argument(
    #     "--output_dir_name",
    #     type=str,
    #     required=True,
    #     default="results/",
    #     help="Directory to save output plots and results (default: 'results/')",
    # )
    # parser.add_argument("--prefix", type=str, required=True, default="results/", help="The prefix for files.")
    # parser.add_argument("--disable_viz", action="store_true", help="Disable vizualization (default: False)")

    # # Parse arguments
    # args = parser.parse_args()

    # Construct paths
    CONFIG_PATH = (
        "/home/tutuna/box_ws/src/grand_tour_box/box_utils/box_auto/cfg/evo_evaluations_GMSF_IMU_comparison.yaml"
    )
    INPUT_FOLDER_PATH = (
        "/home/tutuna/Videos/2024-11-03-13-51-43_eigergletscher_hike_down/2024-11-03-13-51-43_evo_evaluations"
    )
    OUTPUT_DIR_NAME = "/home/tutuna/Videos/2024-11-03-13-51-43_eigergletscher_hike_down/for_paper/"
    DISABLE_VIZ = False
    PREFIX = "tt_"

    # Load the configuration file
    config = load_config(CONFIG_PATH)

    process_directory(INPUT_FOLDER_PATH, OUTPUT_DIR_NAME, config, DISABLE_VIZ, PREFIX)
