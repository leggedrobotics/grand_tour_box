#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import os
import yaml

# from evo.core.trajectory import PoseTrajectory3D
from evo.core.trajectory import Plane
from evo.core import sync
from evo.tools import plot, file_interface
import evo.main_ape as main_ape
import evo.main_rpe as main_rpe
from evo.core.metrics import PoseRelation
from evo.core.metrics import Unit
from evo.tools.settings import SETTINGS
import matplotlib.cm as cm
import matplotlib as mpl
import pandas as pd

SETTINGS.plot_figsize = [12, 8]
SETTINGS.plot_split = True
SETTINGS.plot_usetex = False
SETTINGS.plot_xyz_realistic = False
SETTINGS.plot_show_legend = True
SETTINGS.plot_show_axis = True

# magic plot configuration
import matplotlib.pyplot as plt

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
    time_diffs = []
    time_diffs = np.diff(traj_estimated.timestamps)
    mean_time_diff = np.mean(time_diffs)
    rmse_time_diff = np.sqrt(np.mean(time_diffs**2))
    std_time_diff = np.std(time_diffs)

    plt.figure(figsize=(10, 6))
    # plt.plot(time_diffs, label="Time Differences")
    plt.plot(np.where(time_diffs < 1, time_diffs, np.nan), label="Time Differences < 1s")
    plt.xlabel("Index")
    plt.ylabel("Time Difference (s)")
    plt.title("Time Differences Between Sequential Entries")
    plt.legend()
    plt.grid(True)

    textstr = "\n".join(
        (
            f"Mean: {mean_time_diff:.4f} s",
            f"RMSE: {rmse_time_diff:.4f} s",
            f"STD: {std_time_diff:.4f} s",
        )
    )
    plt.gcf().text(0.5, 0.90, textstr, fontsize=12, ha="center")
    plt.savefig(output_path + "/time_differences_before_" + test_name + ".png", dpi=300)
    plt.close()

    # Filter the trajectories
    if filter_config.get("enable_covariance_based_removal", False):
        # Read the covariances, can be a empty file.
        estimated_file_covs_path = estimated_file[:-4] + "_covariance.tum"
        est_x_cov, est_y_cov, est_z_cov, est_roll_cov, est_pitch_cov, est_yaw_cov = get_covariances(
            estimated_file_covs_path
        )

        if len(est_x_cov) != 0 and len(est_x_cov) != len(traj_estimated.positions_xyz):
            raise ValueError("Size of est_x_cov is not empty and not the same size as traj_estimated.")

        threshold = filter_config.get("covariance_percentile_threshold", 95)

        est_x_cov_indices_to_remove = get_top_n_percentile_indices(est_x_cov, threshold)
        est_y_cov_indices_to_remove = get_top_n_percentile_indices(est_y_cov, threshold)
        est_z_cov_indices_to_remove = get_top_n_percentile_indices(est_z_cov, threshold)
        est_roll_cov_indices_to_remove = get_top_n_percentile_indices(est_roll_cov, threshold)
        est_pitch_cov_indices_to_remove = get_top_n_percentile_indices(est_pitch_cov, threshold)
        est_yaw_cov_indices_to_remove = get_top_n_percentile_indices(est_yaw_cov, threshold)
        all_indices_to_remove = np.unique(
            np.concatenate(
                [
                    est_x_cov_indices_to_remove,
                    est_y_cov_indices_to_remove,
                    est_z_cov_indices_to_remove,
                    est_roll_cov_indices_to_remove,
                    est_pitch_cov_indices_to_remove,
                    est_yaw_cov_indices_to_remove,
                ]
            )
        )

        indices_to_keep = np.setdiff1d(np.arange(len(traj_estimated.positions_xyz)), all_indices_to_remove)
        traj_estimated.reduce_to_ids(indices_to_keep)

    if filter_config.get("enable_no_motion_removal", False):
        # Find indices where the difference to the first value of traj_estimated.positions_xyz is more than 50cm
        distances = np.linalg.norm(traj_estimated.positions_xyz - traj_estimated.positions_xyz[0], axis=1)
        indices_to_keep = np.where(distances > filter_config.get("distance_threshold", 10))[0]
        traj_estimated.reduce_to_ids(indices_to_keep)

    if filter_config.get("ap20_peak_rejection", False):
        # Remove the measurements around the lost AP20 measurements
        trailing_window = filter_config.get("ap20_peak_rejection_trailing_window", 5)
        time_diffs = []
        for i in range(1, len(traj_reference.timestamps)):
            time_diffs.append(traj_reference.timestamps[i] - traj_reference.timestamps[i - 1])
        time_diffs = np.array(time_diffs)

        indices_to_remove = np.where(time_diffs > filter_config.get("ap20_peak_rejection_threshold", 0.1))[0]
        extended_indices_to_remove = set(indices_to_remove)
        for idx in indices_to_remove:
            extended_indices_to_remove.update(range(idx, idx + trailing_window))

            if idx - trailing_window >= 0:
                extended_indices_to_remove.update(range(idx - trailing_window, idx))

        extended_indices_to_remove = np.unique(np.array(sorted(extended_indices_to_remove)))

        # Get the indices to keep
        indices_to_keep = np.setdiff1d(np.arange(len(traj_reference.positions_xyz)), extended_indices_to_remove)

        # Reduce the trajectory to the indices to keep
        traj_reference.reduce_to_ids(indices_to_keep)

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

    results = []
    fig_title = ""
    if mode == "ate":
        save_prefix = "ate"
        fig_title = "Absolute Translation Error (ATE) Trajectory"
        result = main_ape.ape(
            traj_reference,
            traj_estimated,
            est_name=test_name,
            ref_name="AP20",
            pose_relation=PoseRelation.point_distance,
            align=True,
            align_origin=False,
            n_to_align=n_to_align,
            correct_scale=correct_scale,
            project_to_plane=plane_param,
        )
    elif mode == "rte":
        save_prefix = "rte"
        fig_title = "Relative Translation Error (RTE) Trajectory"
        result = main_rpe.rpe(
            traj_reference,
            traj_estimated,
            est_name=test_name,
            ref_name="AP20",
            pose_relation=PoseRelation.point_distance,
            delta=delta,
            delta_unit=delta_unit,
            all_pairs=False,
            align=True,
            correct_scale=correct_scale,
            n_to_align=n_to_align,
            project_to_plane=plane_param,
            support_loop=False,
        )

    results.append(result)

    plt.figure(figsize=(12, 8))
    plt.scatter(
        traj_reference.positions_xyz[:, 0], traj_reference.positions_xyz[:, 1], c="blue", label="Reference", s=4
    )
    plt.scatter(traj_estimated.positions_xyz[:, 0], traj_estimated.positions_xyz[:, 1], c="red", label="Estimated", s=4)
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title("Trajectory Scatter Plot")
    plt.legend()
    plt.grid(True)
    plt.savefig(output_path + "/scatter_plot_" + test_name + ".png", dpi=300)
    plt.close()

    # Plot and save traj_reference.positions_xyz as scatter plot
    fig, axs = plt.subplots(3, 1, figsize=(12, 16))

    # X component scatter plot
    axs[0].scatter(traj_reference.timestamps, traj_reference.positions_xyz[:, 0], c="blue", label="Reference", s=3)
    axs[0].scatter(traj_reference.timestamps, traj_estimated.positions_xyz[:, 0], c="red", label="Estimated", s=3)
    axs[0].set_xlabel("Time [s]")
    axs[0].set_ylabel("X [m]")
    axs[0].set_title("Trajectory X Scatter Plot")
    axs[0].legend()
    axs[0].grid(True)

    # Y component scatter plot
    axs[1].scatter(traj_reference.timestamps, traj_reference.positions_xyz[:, 1], c="blue", label="Reference", s=3)
    axs[1].scatter(traj_reference.timestamps, traj_estimated.positions_xyz[:, 1], c="red", label="Estimated", s=3)
    axs[1].set_xlabel("Time [s]")
    axs[1].set_ylabel("Y [m]")
    axs[1].set_title("Trajectory Y Scatter Plot")
    axs[1].legend()
    axs[1].grid(True)

    # Z component scatter plot
    axs[2].scatter(traj_reference.timestamps, traj_reference.positions_xyz[:, 2], c="blue", label="Reference", s=3)
    axs[2].scatter(traj_reference.timestamps, traj_estimated.positions_xyz[:, 2], c="red", label="Estimated", s=3)
    axs[2].set_xlabel("Time [s]")
    axs[2].set_ylabel("Z [m]")
    axs[2].set_title("Trajectory Z Scatter Plot")
    axs[2].legend()
    axs[2].grid(True)

    plt.tight_layout()
    plt.savefig(output_path + "/scatter_plot_XYZ_" + test_name + ".png", dpi=300)
    plt.close()

    fig = plt.figure(figsize=(16, 12))
    fig.suptitle(fig_title, fontsize=25)
    ax = plot.prepare_axis(fig, plot.PlotMode.xyz)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")

    plot.traj(ax, plot.PlotMode.xyz, traj_reference, style="--", alpha=0.5, label="Ref.")
    plot.traj(ax, plot.PlotMode.xyz, traj_estimated, style="-", alpha=0.5, label=" " + test_name + " ")

    ax.view_init(elev=30, azim=45)

    min_map = result.stats["min"]
    max_map = result.stats["max"]
    pos = traj_estimated.positions_xyz
    norm = mpl.colors.Normalize(vmin=min_map, vmax=max_map, clip=True)
    mapper = cm.ScalarMappable(norm=norm, cmap=SETTINGS.plot_trajectory_cmap)
    array = result.np_arrays["error_array"]
    mapper.set_array(array)
    colors = [mapper.to_rgba(a) for a in array]
    line_collection = plot.colored_line_collection(pos, colors, plot.PlotMode.xyz)
    ax.add_collection(line_collection)
    ax.autoscale_view(True, True, True)
    plot.set_aspect_equal(ax)

    cbar = fig.colorbar(mapper, ticks=[min_map, (max_map - (max_map - min_map) / 2), max_map], ax=ax)
    cbar.ax.set_yticklabels(
        ["{0:0.3f}".format(min_map), "{0:0.3f}".format(max_map - (max_map - min_map) / 2), "{0:0.3f}".format(max_map)]
    )

    cbar.set_label(save_prefix + "[m]", labelpad=20)

    cbar.ax.yaxis.label.set_size(25)
    cbar.ax.tick_params(labelsize=25)
    ax.legend(frameon=True)

    fig.savefig(output_path + "/" + save_prefix + "_3d_plot_" + test_name + ".png", dpi=300)

    file_interface.save_res_file(
        output_path + "/" + save_prefix + "_results_" + test_name + ".zip", results[0], confirm_overwrite=not True
    )

    if mode == "ate":
        print("ATE Results: ")
    if mode == "rte":
        print("RTE Results: ")
    print("Compared_pose_pairs %d pairs" % (len(results[0].np_arrays["error_array"])))
    print("rmse %f m" % results[0].stats["rmse"])
    print("mean %f m" % results[0].stats["mean"])
    print("median %f m" % results[0].stats["median"])
    print("std %f m" % results[0].stats["std"])
    print("min %f m" % results[0].stats["min"])
    print("max %f m" % results[0].stats["max"])

    plot_x_dimension = "seconds"
    # Plot the raw metric values.
    fig_stats = plt.figure(figsize=SETTINGS.plot_figsize)
    if plot_x_dimension == "distances" and "distances_from_start" in results[0].np_arrays:
        x_array = results[0].np_arrays["distances_from_start"]
        x_label = "$d$ (m)"
    elif plot_x_dimension == "seconds" and "seconds_from_start" in results[0].np_arrays:
        x_array = results[0].np_arrays["seconds_from_start"]
        x_label = "$t$ (s)"
    else:
        x_array = None
        x_label = "index"

    plot.error_array(
        fig_stats.gca(),
        results[0].np_arrays["error_array"],
        x_array=x_array,
        statistics={s: results[0].stats[s] for s in SETTINGS.plot_statistics if s not in ("min", "max")},
        name=results[0].info["label"],
        title=results[0].info["title"],
        xlabel=x_label,
    )

    fig_stats.savefig(output_path + "/" + save_prefix + "_statistics_" + test_name + ".png", dpi=300)

    if filter_config.get("disable_viz", False):
        return
    else:
        plt.tight_layout()
        plt.show()


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
        run_eval(test_name, reference_file, estimated_file, params, output_dir, filter_config, "ate")
        run_eval(test_name, reference_file, estimated_file, params, output_dir, filter_config, "rte")


def load_config(config_path):
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
    return config


if __name__ == "__main__":

    import argparse

    parser = argparse.ArgumentParser(description="Run ATE and RTE evaluation for multiple tests.")

    # Define command-line arguments
    parser.add_argument("--config", type=str, required=True, help="Path to the configuration YAML file")
    parser.add_argument(
        "--input_folder_path",
        type=str,
        required=True,
        default="mission_data/",
        help="Mission data subdirectory (default: 'mission_data/')",
    )
    parser.add_argument(
        "--output_dir_name",
        type=str,
        required=True,
        default="results/",
        help="Directory to save output plots and results (default: 'results/')",
    )
    parser.add_argument("--prefix", type=str, required=True, default="results/", help="The prefix for files.")
    parser.add_argument("--disable_viz", action="store_true", help="Disable vizualization (default: False)")

    # Parse arguments
    args = parser.parse_args()

    # Construct paths
    CONFIG_PATH = args.config
    INPUT_FOLDER_PATH = args.input_folder_path
    OUTPUT_DIR_NAME = args.output_dir_name
    DISABLE_VIZ = args.disable_viz
    PREFIX = args.prefix

    # Load the configuration file
    config = load_config(CONFIG_PATH)

    process_directory(INPUT_FOLDER_PATH, OUTPUT_DIR_NAME, config, DISABLE_VIZ, PREFIX)
