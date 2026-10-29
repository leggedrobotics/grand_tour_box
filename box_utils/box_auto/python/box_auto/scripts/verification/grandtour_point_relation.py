#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import yaml
from box_auto.utils.evaluation_config import validate_evaluation_config

# from evo.core.trajectory import PoseTrajectory3D
from evo.core.trajectory import Plane
from evo.core import sync
from evo.core.result import Result
from evo.tools import plot, file_interface
import evo.main_ape as main_ape
import evo.main_rpe as main_rpe
from evo.core.metrics import PoseRelation
from evo.core.metrics import Unit
from evo.tools.settings import SETTINGS
import matplotlib.cm as cm
import matplotlib as mpl

import pandas as pd
import os
import copy

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

MIN_EST_LEN_BEFORE_ASSOCIATION = 100

TRAJECTORY_DISPLAY_NAMES = {
    "ap20_robot": "AP20",
    "cpt7_ie_tc": "CPT7 IE-TC",
    "ie_tc": "CPT7 IE-TC",
    "dlio": "DLIO",
    "fast_livo": "FAST-LIVO",
    "state_estimator": "State estimator",
}

TRAJECTORY_COLORS = {
    "ap20_robot": "tab:orange",
    "ie_tc": "tab:red",
    "dlio": "tab:purple",
    "fast_livo": "tab:red",
    "state_estimator": "tab:green",
}


def trajectory_display_name(name):
    return TRAJECTORY_DISPLAY_NAMES.get(name, name.replace("_", " ").title())


def get_point_relation_reference_name(config):
    reference_names = {
        param_set.get("evaluations", {}).get("point_relation", {}).get("reference_file")
        for param_set in config.get("bag_topic_pairs", [])
        if param_set.get("test_name")
        and param_set.get("evaluations", {}).get("point_relation", {}).get("reference_file")
    }
    if len(reference_names) != 1:
        raise ValueError(
            "Combined plots require exactly one point-relation reference; "
            f"configured references: {sorted(reference_names)}"
        )
    return next(iter(reference_names))


def plot_line_with_time_gaps(ax, timestamps, x_values, y_values, color, label, max_gap=0.25):
    """Plot line segments without bridging missing measurement intervals."""
    timestamps = np.asarray(timestamps)
    split_indices = np.where(np.diff(timestamps) > max_gap)[0] + 1
    starts = np.concatenate(([0], split_indices))
    ends = np.concatenate((split_indices, [len(timestamps)]))

    for segment_index, (start, end) in enumerate(zip(starts, ends)):
        ax.plot(
            x_values[start:end],
            y_values[start:end],
            color=color,
            linewidth=1.5,
            label=label if segment_index == 0 else None,
        )


def _project_positions_for_stationary_tail(positions_xyz, detection_plane):
    if detection_plane == "xy":
        return positions_xyz[:, :2]
    if detection_plane == "xyz":
        return positions_xyz
    print("\033[91mInvalid stationary tail detection plane.\033[0m")
    raise ValueError("Invalid stationary tail detection plane.")


def find_stationary_tail_cutoff(timestamps, positions_xyz, radius, min_duration, keep_duration, detection_plane):
    if len(timestamps) < 2:
        return None, "trajectory has fewer than two timestamps"

    timestamps = np.asarray(timestamps, dtype=np.float64)
    positions_xyz = np.asarray(positions_xyz, dtype=np.float64)
    positions_for_detection = _project_positions_for_stationary_tail(positions_xyz, detection_plane)

    final_position = positions_for_detection[-1]
    distances_to_final_position = np.linalg.norm(positions_for_detection - final_position, axis=1)
    stationary_indices = distances_to_final_position <= radius

    if not stationary_indices[-1]:
        return None, "final trajectory point is not stationary"

    stationary_start_idx = len(stationary_indices) - 1
    while stationary_start_idx > 0 and stationary_indices[stationary_start_idx - 1]:
        stationary_start_idx -= 1

    stationary_tail_duration = timestamps[-1] - timestamps[stationary_start_idx]
    if stationary_tail_duration < min_duration:
        return None, (
            f"stationary tail duration {stationary_tail_duration:.3f}s "
            f"is below minimum {min_duration:.3f}s"
        )

    cutoff_time = timestamps[stationary_start_idx] + keep_duration
    if cutoff_time >= timestamps[-1]:
        return None, (
            f"cutoff {cutoff_time:.6f} would not remove samples "
            f"(trajectory ends at {timestamps[-1]:.6f})"
        )

    return cutoff_time, (
        f"tail_start={timestamps[stationary_start_idx]:.6f}, "
        f"tail_duration={stationary_tail_duration:.3f}s"
    )


def reduce_trajectory_to_cutoff_time(traj, cutoff_time):
    size_before = len(traj.timestamps)
    indices_to_keep = np.where(traj.timestamps <= cutoff_time)[0]
    traj.reduce_to_ids(indices_to_keep)
    size_after = len(traj.timestamps)
    return size_before, size_after


def compute_nearest_neighbor_time_diffs(timestamps_a, timestamps_b):
    ts_a = np.asarray(timestamps_a, dtype=np.float64)
    ts_b = np.asarray(timestamps_b, dtype=np.float64)

    if ts_a.size < 2 or ts_b.size < 2:
        return np.array([])

    # operate on sorted copies to be robust to any ordering issues
    ts_a = np.sort(ts_a)
    ts_b = np.sort(ts_b)

    # restrict to overlapping time interval to avoid edge effects
    t0 = max(ts_a[0], ts_b[0])
    t1 = min(ts_a[-1], ts_b[-1])
    if t1 <= t0:
        return np.array([])

    ts_a = ts_a[(ts_a >= t0) & (ts_a <= t1)]
    ts_b = ts_b[(ts_b >= t0) & (ts_b <= t1)]
    if ts_a.size == 0 or ts_b.size == 0:
        return np.array([])

    # query the larger set with the smaller set
    if ts_a.size <= ts_b.size:
        q = ts_a
        t = ts_b
    else:
        q = ts_b
        t = ts_a

    if t.size < 2:
        return np.array([])

    idx = np.searchsorted(t, q, side="left")
    idx = np.clip(idx, 1, t.size - 1)

    left = np.abs(q - t[idx - 1])
    right = np.abs(q - t[idx])
    return np.minimum(left, right)


def save_failed_result(output_path, zip_file_name, reason):
    """Replace any previous result with an explicit, non-ranking failure record."""
    result = Result()
    result.info.update(status="failed", reason=reason, title="Evaluation failed")
    result.stats.update({key: float("nan") for key in ("mean", "std", "rmse", "min", "max", "median", "sse")})
    result.add_np_array("error_array", np.array([], dtype=float))
    file_interface.save_res_file(os.path.join(output_path, zip_file_name), result, confirm_overwrite=False)


def run_eval(test_name, reference_file, estimated_file, params, output_path, filter_config, mode, disable_viz):

    original_test_name = test_name
    reference_name = os.path.splitext(os.path.basename(reference_file))[0]
    reference_display_name = trajectory_display_name(reference_name)
    reference_is_ap20 = reference_name == "ap20_robot"
    estimated_is_ap20 = original_test_name == "ap20_robot"

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

    zip_file_name = f"{mode}_{test_name}_align-{alignment_post_fix}_results.zip"
    est_name = f"{mode}_{test_name}_align-{alignment_post_fix}"

    test_name += "_point_distances_" + alignment_post_fix

    # Read the trajectories
    traj_reference = file_interface.read_tum_trajectory_file(reference_file)
    traj_estimated = file_interface.read_tum_trajectory_file(estimated_file)
    traj_estimated_full = copy.deepcopy(traj_estimated)

    # Check if the trajectories are valid, first thing to do.
    ref_valid, ref_details = traj_reference.check()
    est_valid, est_details = traj_estimated.check()

    if not ref_valid:
        print(
            f"\033[91mReference trajectory is not valid. Not throwing error but warning the user. Check this. : {ref_details}\033[0m"
        )
    elif reference_is_ap20:
        if filter_config.get("ap20_velocity_rejection", False):
            outliers_indices_to_remove = np.where(
                traj_reference.speeds > filter_config.get("ap20_velocity_rejection_threshold", 3.0)
            )[0]
            if len(outliers_indices_to_remove) != 0:
                print(f"\033[93mRemoving {len(outliers_indices_to_remove)} outlier points with speeds > 3.0 m/s\033[0m")
                indices_to_keep_from_outliers = np.setdiff1d(
                    np.arange(len(traj_reference.positions_xyz)), outliers_indices_to_remove
                )
                traj_reference.reduce_to_ids(indices_to_keep_from_outliers)

    if not est_valid:
        print(
            f"\033[91mEstimated trajectory is not valid. Not throwing error but warning the user. Check this. : {est_details}\033[0m"
        )
    elif estimated_is_ap20:
        if filter_config.get("ap20_velocity_rejection", False):
            outliers_indices_to_remove = np.where(
                traj_estimated.speeds > filter_config.get("ap20_velocity_rejection_threshold", 3.0)
            )[0]
            if len(outliers_indices_to_remove) != 0:
                print(
                    f"\033[93mRemoving {len(outliers_indices_to_remove)} AP20 estimate points "
                    "with speeds above the configured threshold\033[0m"
                )
                indices_to_keep_from_outliers = np.setdiff1d(
                    np.arange(len(traj_estimated.positions_xyz)), outliers_indices_to_remove
                )
                traj_estimated.reduce_to_ids(indices_to_keep_from_outliers)

    if not disable_viz:
        time_differences_file_name = f"{mode}_{test_name}_time_differences.png"
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
        plt.savefig(os.path.join(output_path, time_differences_file_name), dpi=300)
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

    if filter_config.get("ap20_peak_rejection", False) and (reference_is_ap20 or estimated_is_ap20):
        # Remove measurements around lost AP20 measurements, regardless of
        # whether AP20 is configured as the reference or an estimate.
        ap20_trajectory = traj_reference if reference_is_ap20 else traj_estimated
        trailing_window = filter_config.get("ap20_peak_rejection_trailing_window", 5)
        time_diffs = []
        for i in range(1, len(ap20_trajectory.timestamps)):
            time_diffs.append(ap20_trajectory.timestamps[i] - ap20_trajectory.timestamps[i - 1])
        time_diffs = np.array(time_diffs)

        indices_to_remove = np.where(time_diffs > filter_config.get("ap20_peak_rejection_threshold", 0.1))[0]
        extended_indices_to_remove = set(indices_to_remove)
        for idx in indices_to_remove:
            extended_indices_to_remove.update(range(idx, idx + trailing_window))

            if idx - trailing_window >= 0:
                extended_indices_to_remove.update(range(idx - trailing_window, idx))

        extended_indices_to_remove = np.unique(np.array(sorted(extended_indices_to_remove)))

        # Get the indices to keep
        indices_to_keep = np.setdiff1d(np.arange(len(ap20_trajectory.positions_xyz)), extended_indices_to_remove)

        # Reduce only the AP20 trajectory to the indices to keep.
        ap20_trajectory.reduce_to_ids(indices_to_keep)

    if filter_config.get("remove_stationary_tail", False):
        cutoff_time, reason = find_stationary_tail_cutoff(
            traj_reference.timestamps,
            traj_reference.positions_xyz,
            filter_config.get("stationary_tail_radius", 0.5),
            filter_config.get("stationary_tail_min_duration", 20.0),
            filter_config.get("stationary_tail_keep_duration", 0.0),
            filter_config.get("stationary_tail_detection_plane", "xy"),
        )

        if cutoff_time is None:
            print(f"\033[93m[stationary tail] No trim applied: {reason}.\033[0m")
        else:
            reference_end_before_trim = traj_reference.timestamps[-1]
            ref_before, ref_after = reduce_trajectory_to_cutoff_time(traj_reference, cutoff_time)
            est_before, est_after = reduce_trajectory_to_cutoff_time(traj_estimated, cutoff_time)
            removed_duration = max(0.0, reference_end_before_trim - cutoff_time)
            print(
                f"\033[93m[stationary tail] Trimmed at t={cutoff_time:.6f}s ({reason}); "
                f"removed reference samples={ref_before - ref_after}, "
                f"estimated samples={est_before - est_after}, "
                f"removed reference duration~{removed_duration:.3f}s.\033[0m"
            )

    # Save the filtered reference trajectory
    traj_reference_filtered_full = copy.deepcopy(traj_reference)
    reference_file_base = reference_file.replace(".tum", "")
    file_interface.write_tum_trajectory_file(
        os.path.join(output_path, reference_file_base + "_filtered.tum"), traj_reference, False
    )

    sizeOfRererefenceBeforeAssociation = len(traj_reference.timestamps)
    sizeOfEstimatedBeforeAssociation = len(traj_estimated.timestamps)

    # Record unusable estimates explicitly so aggregation retains failed methods.
    if sizeOfEstimatedBeforeAssociation < MIN_EST_LEN_BEFORE_ASSOCIATION:
        print(
            f"\033[91mSkipping {mode.upper()} for '{test_name}': estimated trajectory length "
            f"{sizeOfEstimatedBeforeAssociation} < {MIN_EST_LEN_BEFORE_ASSOCIATION} before association. "
            "Recording a failed evaluation.\033[0m"
        )
        save_failed_result(output_path, zip_file_name, f"Only {sizeOfEstimatedBeforeAssociation} estimated poses; minimum is {MIN_EST_LEN_BEFORE_ASSOCIATION}")
        return

    reference_time_span_before_association = float("nan")
    estimated_time_span_before_association = float("nan")
    reference_rate_before_association = float("nan")
    estimated_rate_before_association = float("nan")

    if sizeOfRererefenceBeforeAssociation >= 2:
        reference_time_span_before_association = traj_reference.timestamps[-1] - traj_reference.timestamps[0]
        if reference_time_span_before_association > 0.0:
            reference_rate_before_association = sizeOfRererefenceBeforeAssociation / reference_time_span_before_association

    if sizeOfEstimatedBeforeAssociation >= 2:
        estimated_time_span_before_association = traj_estimated.timestamps[-1] - traj_estimated.timestamps[0]
        if estimated_time_span_before_association > 0.0:
            estimated_rate_before_association = sizeOfEstimatedBeforeAssociation / estimated_time_span_before_association

    # Check if reference or estimated has more elements
    if len(traj_reference.timestamps) <= len(traj_estimated.timestamps):
        size_of_elements_before_association = len(traj_reference.timestamps)
    else:
        size_of_elements_before_association = len(traj_estimated.timestamps)

    nn_time_diffs = compute_nearest_neighbor_time_diffs(traj_reference.timestamps, traj_estimated.timestamps)
    if len(nn_time_diffs) != 0:
        p50 = np.percentile(nn_time_diffs, 50)
        p90 = np.percentile(nn_time_diffs, 90)
        p95 = np.percentile(nn_time_diffs, 95)
        p99 = np.percentile(nn_time_diffs, 99)
        maxv = np.max(nn_time_diffs)
        frac_within = np.mean(nn_time_diffs <= float(t_max_diff)) * 100.0
        print(f"\033[93m[timestamp NN] t_max_diff (set): {float(t_max_diff):.6f} s | within set: {frac_within:.1f}%\033[0m")
        print(
            f"\033[93m[timestamp NN] recommended t_max_diff: p95={p95:.6f} s | p99={p99:.6f} s "
            f"(p50={p50:.6f}, p90={p90:.6f}, max={maxv:.6f})\033[0m"
        )
    else:
        print(f"\033[93m[timestamp NN] t_max_diff (set): {float(t_max_diff):.6f} s | recommended: N/A (insufficient overlap/data)\033[0m")

    try:
        # Actual association
        traj_reference, traj_estimated = sync.associate_trajectories(
            traj_reference, traj_estimated, t_max_diff, t_offset
        )
    except Exception as e:
        print(f"\033[91mError in association: {e}\033[0m")
        save_failed_result(output_path, zip_file_name, f"Timestamp association failed: {e}")
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
        print(f"\033[91mlen(traj_reference.timestamps): {sizeOfRererefenceBeforeAssociation}\033[0m")
        print(f"\033[91mlen(traj_estimated.timestamps): {sizeOfEstimatedBeforeAssociation}\033[0m")
        print(f"\033[91mReference measurement rate (before association): {reference_rate_before_association:.6f} Hz\033[0m")
        print(f"\033[91mEstimated measurement rate (before association): {estimated_rate_before_association:.6f} Hz\033[0m")
        print(
            f"\033[91mLess than 50% match. Either the association parameters are wrong or "
            f"the {reference_display_name} reference has many missing segments.\033[0m"
        )

    results = []
    fig_title = ""
    if mode == "ate":
        save_prefix = "ate"
        fig_title = "Absolute Translation Error (ATE) Trajectory"
        result = main_ape.ape(
            traj_reference,
            traj_estimated,
            est_name=est_name,
            ref_name=reference_display_name,
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
            est_name=est_name,
            ref_name=reference_display_name,
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

    # Export the exact timestamp-associated pair used by the metric. The
    # estimated trajectory has also been aligned by evo at this point.
    associated_reference_path = os.path.join(
        output_path,
        f"{mode}_{original_test_name}_align-{alignment_post_fix}_associated_{reference_name}.tum",
    )
    associated_estimated_path = os.path.join(
        output_path,
        f"{mode}_{original_test_name}_align-{alignment_post_fix}_associated_{original_test_name}_aligned.tum",
    )
    file_interface.write_tum_trajectory_file(associated_reference_path, traj_reference, False)
    file_interface.write_tum_trajectory_file(associated_estimated_path, traj_estimated, False)
    print(f"Associated reference trajectory: {associated_reference_path}")
    print(f"Associated aligned estimate: {associated_estimated_path}")

    # The ATE alignment is estimated from associated pose pairs. Apply that
    # same transform to the complete raw estimate so it can be inspected
    # without dropping samples during timestamp association.
    traj_estimated_full_aligned = None
    if mode == "ate" and os.path.abspath(estimated_file) != os.path.abspath(reference_file):
        traj_estimated_full_aligned = copy.deepcopy(traj_estimated_full)
        traj_estimated_full_aligned.transform(result.np_arrays["alignment_transformation_sim3"])
        full_raw_aligned_path = os.path.join(
            output_path,
            f"{mode}_{original_test_name}_align-{alignment_post_fix}_full_raw_aligned.tum",
        )
        file_interface.write_tum_trajectory_file(
            full_raw_aligned_path, traj_estimated_full_aligned, False
        )
        print(f"Full raw aligned estimate: {full_raw_aligned_path}")

    if not disable_viz:
        # Match evo_traj's XYZ view: retain the complete estimate and overlay
        # the available filtered reference positions as scatter data.
        if traj_estimated_full_aligned is not None:
            fig, axs = plt.subplots(3, 1, figsize=(16, 14), sharex=True)
            axis_names = ("X", "Y", "Z")
            for axis_index, (ax, axis_name) in enumerate(zip(axs, axis_names)):
                plot_line_with_time_gaps(
                    ax,
                    traj_estimated_full_aligned.timestamps,
                    traj_estimated_full_aligned.timestamps,
                    traj_estimated_full_aligned.positions_xyz[:, axis_index],
                    "tab:blue",
                    f"{original_test_name} (full, aligned)",
                )
                ax.scatter(
                    traj_reference_filtered_full.timestamps,
                    traj_reference_filtered_full.positions_xyz[:, axis_index],
                    color="tab:red",
                    s=10,
                    alpha=0.75,
                    label=f"{reference_display_name} reference positions",
                    zorder=3,
                )
                ax.set_ylabel(f"{axis_name} [m]")
                ax.set_title(f"{axis_name} position")
                ax.grid(True)
                ax.legend()

            axs[-1].set_xlabel("t [s]")
            fig.suptitle(
                f"Full {trajectory_display_name(original_test_name)} trajectory with "
                f"{reference_display_name} reference scatter overlay"
            )
            fig.tight_layout()
            fig.savefig(
                os.path.join(
                    output_path,
                    f"full_{original_test_name}_xyz_with_{reference_name}_reference_scatter.png",
                ),
                dpi=300,
            )
            plt.close(fig)

        fig, ax = plt.subplots(figsize=(12, 8))
        plot_line_with_time_gaps(
            ax,
            traj_reference.timestamps,
            traj_reference.positions_xyz[:, 0],
            traj_reference.positions_xyz[:, 1],
            color="tab:blue",
            label=f"{reference_display_name} line",
        )
        ax.scatter(
            traj_reference.positions_xyz[:, 0],
            traj_reference.positions_xyz[:, 1],
            color="tab:blue",
            s=10,
            alpha=0.65,
            label=f"{reference_display_name} samples",
        )
        plot_line_with_time_gaps(
            ax,
            traj_estimated.timestamps,
            traj_estimated.positions_xyz[:, 0],
            traj_estimated.positions_xyz[:, 1],
            color="tab:red",
            label=f"{original_test_name} line",
        )
        ax.scatter(
            traj_estimated.positions_xyz[:, 0],
            traj_estimated.positions_xyz[:, 1],
            color="tab:red",
            s=10,
            alpha=0.65,
            label=f"{original_test_name} samples",
        )
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.set_title(
            f"{trajectory_display_name(original_test_name)} vs "
            f"{reference_display_name} associated trajectory"
        )
        ax.axis("equal")
        ax.legend()
        ax.grid(True)
        fig.tight_layout()
        fig.savefig(
            os.path.join(
                output_path,
                f"{mode}_line_scatter_xy_{original_test_name}_vs_{reference_name}.png",
            ),
            dpi=300,
        )
        plt.close(fig)

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
        plt.savefig(output_path + "/scatter_plot_" + test_name + ".png", dpi=800)
        plt.close()

        # Plot and save traj_reference.positions_xyz as scatter plot
        fig, axs = plt.subplots(3, 1, figsize=(12, 16))

        # X component scatter plot
        plot_line_with_time_gaps(
            axs[0], traj_reference.timestamps, traj_reference.timestamps, traj_reference.positions_xyz[:, 0], "blue", None
        )
        plot_line_with_time_gaps(
            axs[0], traj_estimated.timestamps, traj_estimated.timestamps, traj_estimated.positions_xyz[:, 0], "red", None
        )
        axs[0].scatter(traj_reference.timestamps, traj_reference.positions_xyz[:, 0], c="blue", label="Reference", s=3)
        axs[0].scatter(traj_estimated.timestamps, traj_estimated.positions_xyz[:, 0], c="red", label="Estimated", s=3)
        axs[0].set_xlabel("Time [s]")
        axs[0].set_ylabel("X [m]")
        axs[0].set_title("Trajectory X Line/Scatter Plot")
        axs[0].legend()
        axs[0].grid(True)

        # Y component scatter plot
        plot_line_with_time_gaps(
            axs[1], traj_reference.timestamps, traj_reference.timestamps, traj_reference.positions_xyz[:, 1], "blue", None
        )
        plot_line_with_time_gaps(
            axs[1], traj_estimated.timestamps, traj_estimated.timestamps, traj_estimated.positions_xyz[:, 1], "red", None
        )
        axs[1].scatter(traj_reference.timestamps, traj_reference.positions_xyz[:, 1], c="blue", label="Reference", s=3)
        axs[1].scatter(traj_estimated.timestamps, traj_estimated.positions_xyz[:, 1], c="red", label="Estimated", s=3)
        axs[1].set_xlabel("Time [s]")
        axs[1].set_ylabel("Y [m]")
        axs[1].set_title("Trajectory Y Line/Scatter Plot")
        axs[1].legend()
        axs[1].grid(True)

        # Z component scatter plot
        plot_line_with_time_gaps(
            axs[2], traj_reference.timestamps, traj_reference.timestamps, traj_reference.positions_xyz[:, 2], "blue", None
        )
        plot_line_with_time_gaps(
            axs[2], traj_estimated.timestamps, traj_estimated.timestamps, traj_estimated.positions_xyz[:, 2], "red", None
        )
        axs[2].scatter(traj_reference.timestamps, traj_reference.positions_xyz[:, 2], c="blue", label="Reference", s=3)
        axs[2].scatter(traj_estimated.timestamps, traj_estimated.positions_xyz[:, 2], c="red", label="Estimated", s=3)
        axs[2].set_xlabel("Time [s]")
        axs[2].set_ylabel("Z [m]")
        axs[2].set_title("Trajectory Z Line/Scatter Plot")
        axs[2].legend()
        axs[2].grid(True)

        plt.tight_layout()
        plt.savefig(
            os.path.join(
                output_path,
                f"{mode}_line_scatter_xyz_time_{original_test_name}_vs_{reference_name}.png",
            ),
            dpi=800,
        )
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

        fig.savefig(output_path + "/" + save_prefix + "_3d_plot_" + test_name + ".png", dpi=800)
    file_interface.save_res_file(os.path.join(output_path, zip_file_name), results[0], confirm_overwrite=not True)

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

    if not disable_viz:
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

        fig_stats.savefig(output_path + "/" + save_prefix + "_statistics_" + test_name + ".png", dpi=800)

    # if disable_viz:
    #     return
    # else:
    #     plt.tight_layout()
    #     plt.show()


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


def save_combined_associated_xyz_plot(output_dir, config):
    """Plot the reference and all associated, aligned estimates on X/Y/Z time axes."""
    reference_name = get_point_relation_reference_name(config)
    reference_display_name = trajectory_display_name(reference_name)
    target_frame = config.get("target_frame", "target")
    reference_path = os.path.join(output_dir, "..", f"{reference_name}_filtered.tum")
    reference_path = os.path.normpath(reference_path)
    if not os.path.isfile(reference_path):
        print(f"Skipping combined XYZ plot: missing {reference_path}")
        return

    reference = file_interface.read_tum_trajectory_file(reference_path)
    estimates = []

    for param_set in config.get("bag_topic_pairs", []):
        test_name = param_set.get("test_name")
        evaluations = param_set.get("evaluations", {})
        configured_reference_name = evaluations.get("point_relation", {}).get("reference_file")
        if not test_name or not configured_reference_name or test_name == configured_reference_name:
            continue

        n_to_align = evaluations.get("parameters", {}).get("n_to_align", -1)
        alignment_postfix = "all" if n_to_align == -1 else str(n_to_align)
        estimate_path = os.path.join(
            output_dir,
            f"ate_{test_name}_align-{alignment_postfix}_associated_{test_name}_aligned.tum",
        )
        if not os.path.isfile(estimate_path):
            print(f"Skipping {test_name} in combined XYZ plot: missing {estimate_path}")
            continue

        estimates.append(
            (
                file_interface.read_tum_trajectory_file(estimate_path),
                TRAJECTORY_COLORS.get(test_name),
                trajectory_display_name(test_name),
            )
        )

    if not estimates:
        print("Skipping combined XYZ plot: no aligned estimate trajectories found")
        return

    fig, axes = plt.subplots(3, 1, figsize=(16, 13), sharex=True)
    for axis_index, (axis, axis_name) in enumerate(zip(axes, ("X", "Y", "Z"))):
        axis.scatter(
            reference.timestamps,
            reference.positions_xyz[:, axis_index],
            color="tab:blue",
            s=10,
            alpha=0.75,
            label=f"{reference_display_name} reference",
            zorder=3,
        )

        for trajectory, color, label in estimates:
            plot_line_with_time_gaps(
                axis,
                trajectory.timestamps,
                trajectory.timestamps,
                trajectory.positions_xyz[:, axis_index],
                color,
                label,
            )
            axis.scatter(
                trajectory.timestamps,
                trajectory.positions_xyz[:, axis_index],
                color=color,
                s=8,
                alpha=0.7,
                zorder=2,
            )

        axis.set_title(f"Trajectory {axis_name} Line/Scatter Plot")
        axis.set_ylabel(f"{axis_name} [m]")
        axis.grid(True)
        axis.legend()

    axes[-1].set_xlabel("Time [s]")
    fig.suptitle(
        f"{reference_display_name} Reference vs Aligned Estimates "
        f"(Associated, {target_frame} Frame)",
        fontsize=16,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    output_path = os.path.join(
        output_dir,
        f"ate_line_scatter_xyz_time_all_methods_vs_{reference_name}.png",
    )
    fig.savefig(output_path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved combined XYZ line/scatter plot to {output_path}")


def save_combined_raw_aligned_xyz_plot(output_dir, config):
    """Plot complete raw trajectories after alignment and before association."""
    reference_name = get_point_relation_reference_name(config)
    reference_display_name = trajectory_display_name(reference_name)
    target_frame = config.get("target_frame", "target")
    reference_path = os.path.normpath(os.path.join(output_dir, "..", f"{reference_name}.tum"))
    if not os.path.isfile(reference_path):
        print(f"Skipping raw aligned XYZ plot: missing {reference_path}")
        return

    reference = file_interface.read_tum_trajectory_file(reference_path)
    estimates = []
    for param_set in config.get("bag_topic_pairs", []):
        test_name = param_set.get("test_name")
        evaluations = param_set.get("evaluations", {})
        configured_reference_name = evaluations.get("point_relation", {}).get("reference_file")
        if not test_name or not configured_reference_name or test_name == configured_reference_name:
            continue

        n_to_align = evaluations.get("parameters", {}).get("n_to_align", -1)
        alignment_postfix = "all" if n_to_align == -1 else str(n_to_align)
        estimate_path = os.path.join(
            output_dir,
            f"ate_{test_name}_align-{alignment_postfix}_full_raw_aligned.tum",
        )
        if not os.path.isfile(estimate_path):
            print(f"Skipping {test_name} in raw aligned XYZ plot: missing {estimate_path}")
            continue

        estimates.append(
            (
                file_interface.read_tum_trajectory_file(estimate_path),
                TRAJECTORY_COLORS.get(test_name),
                f"{trajectory_display_name(test_name)} (raw, aligned)",
            )
        )

    if not estimates:
        print("Skipping raw aligned XYZ plot: no complete aligned trajectories found")
        return

    fig, axes = plt.subplots(3, 1, figsize=(16, 13), sharex=True)
    for axis_index, (axis, axis_name) in enumerate(zip(axes, ("X", "Y", "Z"))):
        axis.scatter(
            reference.timestamps,
            reference.positions_xyz[:, axis_index],
            color="tab:blue",
            s=10,
            alpha=0.75,
            label=f"{reference_display_name} reference (raw)",
            zorder=3,
        )

        for trajectory, color, label in estimates:
            plot_line_with_time_gaps(
                axis,
                trajectory.timestamps,
                trajectory.timestamps,
                trajectory.positions_xyz[:, axis_index],
                color,
                label,
            )
            axis.scatter(
                trajectory.timestamps,
                trajectory.positions_xyz[:, axis_index],
                color=color,
                s=5,
                alpha=0.55,
                zorder=2,
            )

        axis.set_title(f"Raw Trajectory {axis_name} After Alignment, Before Association")
        axis.set_ylabel(f"{axis_name} [m]")
        axis.grid(True)
        axis.legend()

    axes[-1].set_xlabel("Time [s]")
    fig.suptitle(
        f"Raw {reference_display_name} and Estimate Trajectories in {target_frame} "
        "(ATE SE(3) Alignment Applied Before Association Filtering)",
        fontsize=16,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    output_path = os.path.join(
        output_dir,
        f"ate_line_scatter_xyz_time_all_methods_raw_aligned_before_association_vs_{reference_name}.png",
    )
    fig.savefig(output_path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved raw aligned XYZ line/scatter plot to {output_path}")


def process_directory(base_path, output_dir, config, disable_viz, prefix):

    filter_config = {
        "enable_covariance_based_removal": config["filter_by_covariance"],
        "covariance_percentile_threshold": config["covariance_percentile_threshold"],
        "enable_no_motion_removal": config["remove_until_motion"],
        "distance_threshold": config["distance_threshold"],
        "ap20_peak_rejection": config["ap20_peak_rejection"],
        "ap20_velocity_rejection": config["ap20_velocity_rejection"],
        "ap20_velocity_rejection_threshold": config["ap20_velocity_rejection_threshold"],
        "ap20_peak_rejection_threshold": config["ap20_peak_rejection_threshold"],
        "ap20_peak_rejection_trailing_window": config["ap20_peak_rejection_trailing_window"],
        "project_to_plane": config["project_to_plane"],
        "remove_stationary_tail": config.get("remove_stationary_tail", False),
        "stationary_tail_radius": config.get("stationary_tail_radius", 0.5),
        "stationary_tail_min_duration": config.get("stationary_tail_min_duration", 20.0),
        "stationary_tail_keep_duration": config.get("stationary_tail_keep_duration", 0.0),
        "stationary_tail_detection_plane": config.get("stationary_tail_detection_plane", "xy"),
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
        run_eval(test_name, reference_file, estimated_file, params, output_dir, filter_config, "ate", disable_viz)
        run_eval(test_name, reference_file, estimated_file, params, output_dir, filter_config, "rte", disable_viz)

    if not disable_viz:
        save_combined_associated_xyz_plot(output_dir, config)
        save_combined_raw_aligned_xyz_plot(output_dir, config)


def load_config(config_path):
    with open(config_path, "r") as f:
        config = validate_evaluation_config(yaml.safe_load(f))
    return config


def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ("yes", "true", "t", "1"):
        return True
    if v.lower() in ("no", "false", "f", "0"):
        return False
    raise argparse.ArgumentTypeError("Boolean value expected.")


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
    parser.add_argument(
        "--disable_viz",
        type=str2bool,
        nargs="?",
        const=True,
        default=False,
        help="Disable visualization (default: False)",
    )

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

    # Big debug print block (green)
    GREEN = "\033[92m"
    RESET = "\033[0m"

    def _g(label, value):
        print(f"{GREEN}{label:<22}: {value}{RESET}")

    print(f"{GREEN}{'=' * 90}{RESET}")
    print(f"{GREEN}{' GRANDTOUR POINT RELATION - RUN CONFIG ' :^90}{RESET}")
    print(f"{GREEN}{'=' * 90}{RESET}")

    _g("CONFIG_PATH", CONFIG_PATH)
    _g("INPUT_FOLDER_PATH", INPUT_FOLDER_PATH)
    _g("OUTPUT_DIR_NAME", OUTPUT_DIR_NAME)
    _g("DISABLE_VIZ", DISABLE_VIZ)
    _g("PREFIX", PREFIX)

    print(f"{GREEN}{'-' * 90}{RESET}")

    process_directory(INPUT_FOLDER_PATH, OUTPUT_DIR_NAME, config, DISABLE_VIZ, PREFIX)
