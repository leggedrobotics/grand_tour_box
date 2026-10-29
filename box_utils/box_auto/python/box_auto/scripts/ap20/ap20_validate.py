import rosbag
from pathlib import Path
from box_auto.utils.atomic_file import atomic_output
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from box_auto.utils import get_bag, upload_bag, MISSION_DATA


"""
Exit Codes:

EXIT CODE 0: Successful Verification
EXIT CODE 1: Totalstation position or prism moved
"""


# =========================
# Hard-coded parameters
# =========================

# Bag selection / naming
INPUT_BAG_GLOB = "*_jetson_ap20_synced.bag"
OUTPUT_BAG_SUFFIX_REPLACE_FROM = "_synced.bag"
OUTPUT_BAG_SUFFIX_REPLACE_TO = "_robot.bag"

# Topics
TOPIC_PRISM_POSITION = "/gt_box/ap20/prism_position"

# Verification settings
MEDIAN_WINDOW_SIZE = 20  # first/last N messages used for median check
MIN_MESSAGES_FOR_CHECK = 40  # must be at least 2 * MEDIAN_WINDOW_SIZE
MEDIAN_DIFF_THRESHOLD_M = 0.003  # 3 mm

# Start-removal settings
REMOVE_START_DISTANCE_THRESHOLD_M = 0.2  # 20 cm

# Plot settings
PLOT_MARKER_SIZE = 4
PLOT_START_MARKER_SIZE = 400
PLOT_START_MARKER_LINEWIDTH = 3
PLOT_END_MARKER_SIZE = 400
PLOT_END_MARKER_LINEWIDTH = 3
PLOT_DPI = 1000
PLOT_PAD_INCHES = 0.02

# Output bag writing
OUTPUT_BAG_COMPRESSION = "lz4"

# ANSI colors
ANSI_GREEN = "\033[92m"
ANSI_RESET = "\033[0m"

# Peak trimming (pre-processing before writing)
ENABLE_PEAK_TRIM_FOR_WRITING = True
PEAK_DELTA_THRESHOLD_M = 3.0  # "peak" if consecutive delta norm exceeds this (first/last peak trim)

# Peak exclusion (additional pre-processing before writing)
ENABLE_PEAK_EXCLUDE_FOR_WRITING = False
PEAK_EXCLUDE_THRESHOLD_M = 1.0  # exclude around any delta peak above this
PEAK_EXCLUDE_RADIUS_IDX = 5  # exclude idx +- this radius around the peak (message index space)

# Time-gap cleaning (additional pre-processing before writing)
ENABLE_TIME_GAP_EXCLUDE_FOR_WRITING = True
TIME_GAP_THRESHOLD_SEC = 0.2  # gap if consecutive header stamps differ by more than this
TIME_GAP_EXCLUDE_RADIUS_IDX = 5  # exclude idx +- this radius around the gap (message index space)

# Post-write verification plots
ENABLE_POSTWRITE_DELTA_PLOT = True
POSTWRITE_DELTA_PLOT_SUFFIX = "_prism_delta_norm_postwrite.png"

ENABLE_POSTWRITE_DT_HDR_PLOT = True
POSTWRITE_DT_HDR_PLOT_SUFFIX = "_prism_dt_header_postwrite.png"

ENABLE_POSTWRITE_DT_AND_X_STACKED_PLOT = True
POSTWRITE_DT_AND_X_STACKED_PLOT_SUFFIX = "_prism_dt_and_x_postwrite.png"

# Post-write stacked plot (x scatter) styling
POSTWRITE_X_SCATTER_MARKER_SIZE = 2
POSTWRITE_X_SCATTER_MARKER_STYLE = "_"

# Debug prints for cleaning decisions
ENABLE_CLEANING_SUMMARY_PRINTS = True


def compute_delta_norm_and_time_from_msgs(prism_positions):
    xs_all = np.array([msg.point.x for msg, _ in prism_positions], dtype=np.float64)
    ys_all = np.array([msg.point.y for msg, _ in prism_positions], dtype=np.float64)
    zs_all = np.array([msg.point.z for msg, _ in prism_positions], dtype=np.float64)

    hdr_ts_all = np.array([msg.header.stamp.to_sec() for msg, _ in prism_positions], dtype=np.float64)
    if hdr_ts_all.size > 0:
        hdr_t_rel = hdr_ts_all - hdr_ts_all[0]
    else:
        hdr_t_rel = np.array([], dtype=np.float64)

    if xs_all.size >= 2:
        dx = np.diff(xs_all)
        dy = np.diff(ys_all)
        dz = np.diff(zs_all)
        delta_norm = np.sqrt(dx * dx + dy * dy + dz * dz)
        delta_t = hdr_t_rel[1:]
    else:
        delta_norm = np.array([], dtype=np.float64)
        delta_t = np.array([], dtype=np.float64)

    return delta_t, delta_norm


def compute_dt_header_and_x_from_msgs(prism_positions):
    hdr_ts_all = np.array([msg.header.stamp.to_sec() for msg, _ in prism_positions], dtype=np.float64)
    xs_all = np.array([msg.point.x for msg, _ in prism_positions], dtype=np.float64)

    if hdr_ts_all.size > 0:
        hdr_t_rel = hdr_ts_all - hdr_ts_all[0]
    else:
        hdr_t_rel = np.array([], dtype=np.float64)

    if hdr_ts_all.size >= 2:
        dt_hdr = np.diff(hdr_ts_all)
        dt_t = hdr_t_rel[1:]
    else:
        dt_hdr = np.array([], dtype=np.float64)
        dt_t = np.array([], dtype=np.float64)

    return hdr_t_rel, xs_all, dt_t, dt_hdr


def save_line_plot(x, y, base_bag_path, suffix, title, xlabel, ylabel):
    plot_path = base_bag_path
    if plot_path.endswith(".bag"):
        plot_path = plot_path[:-4] + suffix
    else:
        plot_path = plot_path + suffix

    fig = plt.figure()
    plt.plot(x, y)
    plt.grid(True)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.tight_layout()
    plt.savefig(plot_path, dpi=PLOT_DPI, bbox_inches="tight", pad_inches=PLOT_PAD_INCHES)
    plt.close(fig)


def save_delta_plot(delta_t, delta_norm, base_bag_path, suffix, title):
    save_line_plot(
        delta_t,
        delta_norm,
        base_bag_path=base_bag_path,
        suffix=suffix,
        title=title,
        xlabel="t_rel (header.stamp) [s]",
        ylabel="||p[i] - p[i-1]|| [m]",
    )


def save_dt_and_x_stacked_plot(t_rel, x_vals, dt_t, dt_hdr, base_bag_path, suffix):
    plot_path = base_bag_path
    if plot_path.endswith(".bag"):
        plot_path = plot_path[:-4] + suffix
    else:
        plot_path = plot_path + suffix

    fig, (ax0, ax1) = plt.subplots(2, 1, sharex=True)

    ax0.scatter(
        t_rel,
        x_vals,
        s=POSTWRITE_X_SCATTER_MARKER_SIZE,
        marker=POSTWRITE_X_SCATTER_MARKER_STYLE,
    )
    ax0.grid(True)
    ax0.set_ylabel("x [m]")
    ax0.set_title("Post-write: x measurements and dt(header.stamp)")

    ax1.plot(dt_t, dt_hdr)
    ax1.grid(True)
    ax1.set_xlabel("t_rel (header.stamp) [s]")
    ax1.set_ylabel("dt(header.stamp) [s]")

    fig.tight_layout()
    fig.savefig(plot_path, dpi=PLOT_DPI, bbox_inches="tight", pad_inches=PLOT_PAD_INCHES)
    plt.close(fig)


def mask_count(mask):
    return int(np.size(mask) - np.count_nonzero(mask))


def process_prism_position_bag(input_bag_path, output_bag_path, skip_check, remove_start, return_check_only):
    if Path(input_bag_path).resolve() == Path(output_bag_path).resolve():
        raise ValueError("AP20 output must differ from the input bag")
    GREEN = ANSI_GREEN
    RESET = ANSI_RESET

    with rosbag.Bag(input_bag_path) as input_bag:
        prism_positions = []
        for _, msg, t in input_bag.read_messages(topics=[TOPIC_PRISM_POSITION]):
            prism_positions.append((msg, t))

        if not prism_positions:
            print("No prism position messages found!")
            return 1, [np.inf, np.inf, np.inf]

        delta_t, delta_norm = compute_delta_norm_and_time_from_msgs(prism_positions)
        save_delta_plot(
            delta_t,
            delta_norm,
            input_bag_path,
            suffix="_prism_delta_norm.png",
            title="Prism consecutive delta norm",
        )

        n_msgs = len(prism_positions)

        keep_mask = np.ones(n_msgs, dtype=bool)
        removed_by_trim = np.zeros(n_msgs, dtype=bool)
        removed_by_peak_exclude = np.zeros(n_msgs, dtype=bool)
        removed_by_time_gap = np.zeros(n_msgs, dtype=bool)

        if ENABLE_PEAK_TRIM_FOR_WRITING and delta_norm.size > 0:
            peak_indices_3m = np.where(delta_norm > PEAK_DELTA_THRESHOLD_M)[0]
            # Trimming between peaks requires two boundaries. A single jump
            # cannot identify which side is valid; retain it for diagnostics.
            if peak_indices_3m.size >= 2:
                first_peak_idx = int(peak_indices_3m[0])
                last_peak_idx = int(peak_indices_3m[-1])

                start_write_idx = first_peak_idx + 1
                end_write_idx = last_peak_idx

                start_write_idx = max(0, min(start_write_idx, n_msgs))
                end_write_idx = max(-1, min(end_write_idx, n_msgs - 1))

                if start_write_idx <= end_write_idx:
                    trim_mask = np.ones(n_msgs, dtype=bool)
                    trim_mask[:start_write_idx] = False
                    trim_mask[end_write_idx + 1 :] = False

                    removed_now = keep_mask & (~trim_mask)
                    removed_by_trim |= removed_now
                    keep_mask &= trim_mask
                else:
                    removed_by_trim |= keep_mask
                    keep_mask[:] = False

        if ENABLE_PEAK_EXCLUDE_FOR_WRITING and delta_norm.size > 0 and n_msgs > 0:
            peak_indices_1m = np.where(delta_norm > PEAK_EXCLUDE_THRESHOLD_M)[0]
            if peak_indices_1m.size > 0:
                for k in peak_indices_1m:
                    k = int(k)
                    lo = max(0, k - PEAK_EXCLUDE_RADIUS_IDX)
                    hi = min(n_msgs - 1, (k + 1) + PEAK_EXCLUDE_RADIUS_IDX)
                    exc_mask = np.ones(n_msgs, dtype=bool)
                    exc_mask[lo : hi + 1] = False

                    removed_now = keep_mask & (~exc_mask)
                    removed_by_peak_exclude |= removed_now
                    keep_mask &= exc_mask

        if ENABLE_TIME_GAP_EXCLUDE_FOR_WRITING and n_msgs >= 2:
            hdr_ts_all = np.array([msg.header.stamp.to_sec() for msg, _ in prism_positions], dtype=np.float64)
            dt_hdr = np.diff(hdr_ts_all)
            gap_indices = np.where(dt_hdr > TIME_GAP_THRESHOLD_SEC)[0]
            if gap_indices.size > 0:
                for k in gap_indices:
                    k = int(k)
                    lo = max(0, k - TIME_GAP_EXCLUDE_RADIUS_IDX)
                    hi = min(n_msgs - 1, (k + 1) + TIME_GAP_EXCLUDE_RADIUS_IDX)
                    gap_mask = np.ones(n_msgs, dtype=bool)
                    gap_mask[lo : hi + 1] = False

                    removed_now = keep_mask & (~gap_mask)
                    removed_by_time_gap |= removed_now
                    keep_mask &= gap_mask

        prism_positions_to_write = [prism_positions[i] for i in range(n_msgs) if keep_mask[i]]

        if ENABLE_CLEANING_SUMMARY_PRINTS:
            total_removed = mask_count(keep_mask)
            kept = int(np.count_nonzero(keep_mask))
            print(f"[CLEAN] total msgs in: {n_msgs}, kept: {kept}, removed: {total_removed}")

            if ENABLE_PEAK_TRIM_FOR_WRITING:
                peak3_cnt = int(np.count_nonzero(delta_norm > PEAK_DELTA_THRESHOLD_M)) if delta_norm.size > 0 else 0
                print(
                    f"[CLEAN] PEAK_TRIM (> {PEAK_DELTA_THRESHOLD_M} m): peaks={peak3_cnt}, removed={int(np.count_nonzero(removed_by_trim))}"
                )
            else:
                print("[CLEAN] PEAK_TRIM: disabled")

            if ENABLE_PEAK_EXCLUDE_FOR_WRITING:
                peak1_cnt = int(np.count_nonzero(delta_norm > PEAK_EXCLUDE_THRESHOLD_M)) if delta_norm.size > 0 else 0
                print(
                    f"[CLEAN] PEAK_EXCLUDE (> {PEAK_EXCLUDE_THRESHOLD_M} m, radius={PEAK_EXCLUDE_RADIUS_IDX}): peaks={peak1_cnt}, removed={int(np.count_nonzero(removed_by_peak_exclude))}"
                )
            else:
                print("[CLEAN] PEAK_EXCLUDE: disabled")

            if ENABLE_TIME_GAP_EXCLUDE_FOR_WRITING and n_msgs >= 2:
                hdr_ts_all = np.array([msg.header.stamp.to_sec() for msg, _ in prism_positions], dtype=np.float64)
                dt_hdr = np.diff(hdr_ts_all)
                gap_cnt = int(np.count_nonzero(dt_hdr > TIME_GAP_THRESHOLD_SEC))
                print(
                    f"[CLEAN] TIME_GAP_EXCLUDE (> {TIME_GAP_THRESHOLD_SEC} s, radius={TIME_GAP_EXCLUDE_RADIUS_IDX}): gaps={gap_cnt}, removed={int(np.count_nonzero(removed_by_time_gap))}"
                )
            else:
                print("[CLEAN] TIME_GAP_EXCLUDE: disabled")

        xs = np.array([msg.point.x for msg, _ in prism_positions], dtype=np.float64)
        ys = np.array([msg.point.y for msg, _ in prism_positions], dtype=np.float64)

        ts = np.array([t.to_sec() for _, t in prism_positions], dtype=np.float64)
        if ts.size > 1:
            tmin = ts.min()
            tmax = ts.max()
            denom = (tmax - tmin) if (tmax - tmin) > 0 else 1.0
            tnorm = (ts - tmin) / denom
        else:
            tnorm = np.zeros_like(ts)

        plot_path = input_bag_path
        if plot_path.endswith(".bag"):
            plot_path = plot_path[:-4] + "_prism_xy.png"
        else:
            plot_path = plot_path + "_prism_xy.png"

        fig = plt.figure()
        sc = plt.scatter(xs, ys, marker="o", s=PLOT_MARKER_SIZE, c=tnorm, cmap="turbo", rasterized=True)

        cbar = plt.colorbar(sc)
        cbar.set_ticks([])
        cbar.ax.text(0.5, -0.02, "EARLIEST", ha="center", va="top", transform=cbar.ax.transAxes)
        cbar.ax.text(0.5, 1.02, "LATEST", ha="center", va="bottom", transform=cbar.ax.transAxes)

        plt.scatter(
            [xs[0]],
            [ys[0]],
            marker="x",
            c="red",
            s=PLOT_START_MARKER_SIZE,
            linewidths=PLOT_START_MARKER_LINEWIDTH,
            zorder=5,
        )

        plt.scatter(
            [xs[-1]],
            [ys[-1]],
            marker="o",
            facecolors="none",
            edgecolors="blue",
            s=PLOT_END_MARKER_SIZE,
            linewidths=PLOT_END_MARKER_LINEWIDTH,
            zorder=5,
        )

        plt.gca().set_aspect("equal", adjustable="box")
        plt.grid(True)
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.title("Prism positions (X-Y)")
        plt.tight_layout()
        plt.savefig(plot_path, dpi=PLOT_DPI, bbox_inches="tight", pad_inches=PLOT_PAD_INCHES)
        plt.close(fig)

        first_position = prism_positions[0][0]

        if len(prism_positions) < MIN_MESSAGES_FOR_CHECK:
            return 2, [np.inf, np.inf, np.inf]

        first_n_msgs = [msg for msg, t in prism_positions[:MEDIAN_WINDOW_SIZE]]
        last_n_msgs = [msg for msg, t in prism_positions[-MEDIAN_WINDOW_SIZE:]]

        first_n_medians = np.median([[msg.point.x, msg.point.y, msg.point.z] for msg in first_n_msgs], axis=0)
        last_n_medians = np.median([[msg.point.x, msg.point.y, msg.point.z] for msg in last_n_msgs], axis=0)

        median_diff = np.abs(first_n_medians - last_n_medians)
        print(
            f"{GREEN}Median difference (x, y, z): {median_diff} -  Number of Prism Positions: {len(prism_positions)}{RESET}"
        )

        if return_check_only:
            return 0, median_diff

        if not np.all(median_diff < MEDIAN_DIFF_THRESHOLD_M) and not skip_check:
            print(
                f"Error {MISSION_DATA}: Median differences exceed {MEDIAN_DIFF_THRESHOLD_M} m! - Nothing will be written to robot.bag"
            )
            return 4, median_diff

        if remove_start:
            prism_positions_to_write = [
                (msg, t) for msg, t in prism_positions_to_write
                if np.linalg.norm([
                    msg.point.x - first_position.point.x,
                    msg.point.y - first_position.point.y,
                    msg.point.z - first_position.point.z,
                ]) > REMOVE_START_DISTANCE_THRESHOLD_M
            ]
        if not prism_positions_to_write:
            print("Error: AP20 filtering retained no positions; existing output is preserved.")
            return 5, median_diff

        print(f"{GREEN}Writing normally to robot bag{RESET}")
        with atomic_output(output_bag_path) as temporary, rosbag.Bag(str(temporary), "w", compression=OUTPUT_BAG_COMPRESSION) as output_bag:
            for msg, t in prism_positions_to_write:
                output_bag.write(TOPIC_PRISM_POSITION, msg, t)

        out_positions = []
        if ENABLE_POSTWRITE_DELTA_PLOT or ENABLE_POSTWRITE_DT_HDR_PLOT or ENABLE_POSTWRITE_DT_AND_X_STACKED_PLOT:
            with rosbag.Bag(output_bag_path) as out_bag:
                for _, msg, t in out_bag.read_messages(topics=[TOPIC_PRISM_POSITION]):
                    out_positions.append((msg, t))

        if ENABLE_POSTWRITE_DELTA_PLOT:
            out_delta_t, out_delta_norm = compute_delta_norm_and_time_from_msgs(out_positions)
            save_delta_plot(
                out_delta_t,
                out_delta_norm,
                output_bag_path,
                suffix=POSTWRITE_DELTA_PLOT_SUFFIX,
                title="Prism consecutive delta norm (post-write)",
            )

        if ENABLE_POSTWRITE_DT_HDR_PLOT:
            out_hdr_t_rel, out_xs, out_dt_t, out_dt_hdr = compute_dt_header_and_x_from_msgs(out_positions)
            save_line_plot(
                out_dt_t,
                out_dt_hdr,
                base_bag_path=output_bag_path,
                suffix=POSTWRITE_DT_HDR_PLOT_SUFFIX,
                title="Consecutive header.stamp dt (post-write)",
                xlabel="t_rel (header.stamp) [s]",
                ylabel="dt(header.stamp) [s]",
            )
        else:
            out_hdr_t_rel, out_xs, out_dt_t, out_dt_hdr = compute_dt_header_and_x_from_msgs(out_positions)

        if ENABLE_POSTWRITE_DT_AND_X_STACKED_PLOT:
            save_dt_and_x_stacked_plot(
                out_hdr_t_rel,
                out_xs,
                out_dt_t,
                out_dt_hdr,
                base_bag_path=output_bag_path,
                suffix=POSTWRITE_DT_AND_X_STACKED_PLOT_SUFFIX,
            )

        upload_bag(output_bag_path)
        return 0, median_diff


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Perform verification.")
    args = parser.parse_args()

    input_bag_path = get_bag(INPUT_BAG_GLOB)
    output_bag_path = input_bag_path.replace(OUTPUT_BAG_SUFFIX_REPLACE_FROM, OUTPUT_BAG_SUFFIX_REPLACE_TO)

    args.use_metadata = False
    args.skip_check = False
    args.remove_start = False

    exit_code, media_diff = process_prism_position_bag(
        input_bag_path, output_bag_path, args.skip_check, args.remove_start, return_check_only=False
    )

    exit(exit_code)
