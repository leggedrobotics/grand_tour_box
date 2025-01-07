#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import argparse
from evo.core.trajectory import PoseTrajectory3D
from evo.core import sync
from evo.tools import plot, file_interface, log
import evo.main_ape as main_ape
import evo.main_rpe as main_rpe
from evo.core.metrics import PoseRelation
from evo.core.metrics import Unit
import evo.common_ape_rpe as common
from evo.tools.settings import SETTINGS
from evo.tools import file_interface
from rosbags.rosbag1 import Reader as Rosbag1Reader
import os
from pathlib import Path
import shutil
import psutil


# temporarily override some package settings
SETTINGS.plot_figsize = [12, 8]
SETTINGS.plot_split = True
SETTINGS.plot_usetex = False

# magic plot configuration
import matplotlib.pyplot as plt

def get_bag(directory, pattern):
    files = [str(s) for s in Path(directory).rglob(pattern)]
    if len(files) != 1:
        print(f"Error: More or less matching bag files found: {pattern} in directory {directory}")
        return [], False

    return files[0], True

def run_ate(args):  

    # Base path of the data
    base_path = args.base_path

    if not base_path.endswith('/'):
        base_path += '/'

    traj_reference = None
    traj_estimated = None


    pattern = "*_ap20_synced.bag"
    f, suc = get_bag(base_path, pattern)
    if suc:
        timestamp = str(f).split("/")[-1]
        timestamp = timestamp.split("_")[0]
    else:
        raise ValueError("Pattern not found: ", pattern, " in Directory")
    bag = Rosbag1Reader(f)  # type: ignore
    bag.open()
    traj_reference = file_interface.read_bag_trajectory(bag, "/gt_box/ap20/prism_position_odometry", cache_tf_tree=False)
    bag.close()



    test_names = ["dlio", "zed2i", "open3d_slam"]
    results = []

    for test_name in test_names:
        if test_name == "dlio":
            # DLIO
            pattern = "*_dlio.bag"
            f, suc = get_bag(base_path, pattern)
            if suc:
                timestamp = str(f).split("/")[-1]
                timestamp = timestamp.split("_")[0]
            else:
                raise ValueError("Pattern not found: ", pattern, " in Directory")
            bag = Rosbag1Reader(f)  # type: ignore
            bag.open()
            traj_estimated = file_interface.read_bag_trajectory(bag, "/dlio/lidar_map_odometry", cache_tf_tree=False)
            bag.close()

        elif test_name == "zed2i":
            # ZED2I
            pattern = "*_zed2i_proprioceptive.bag"
            f, suc = get_bag(base_path, pattern)
            if suc:
                timestamp = str(f).split("/")[-1]
                timestamp = timestamp.split("_")[0]
            else:
                raise ValueError("Pattern not found: ", pattern, " in Directory")
            bag = Rosbag1Reader(f)  # type: ignore
            bag.open()
            # In zed2i_base_link frame
            traj_estimated = file_interface.read_bag_trajectory(bag, "/gt_box/zed2i/zed_node/odom", cache_tf_tree=False)
            bag.close()

        elif test_name == "open3d_slam":
            # Open3d SLAM
            # in hesai_lidar frame
            pattern = "*_open3d_slam.bag"
            f, suc = get_bag(base_path, pattern)
            if suc:
                timestamp = str(f).split("/")[-1]
                timestamp = timestamp.split("_")[0]
            else:
                raise ValueError("Pattern not found: ", pattern, " in Directory")
            bag = Rosbag1Reader(f)  # type: ignore
            bag.open()
            traj_estimated = file_interface.read_bag_trajectory(bag, "/open3d_slam/optimized_poses_as_odometry", cache_tf_tree=False)
            bag.close()

        """ PARAMETERS """
        correct_scale = False
        t_max_diff = 0.02 # in seconds
        t_offset = 0.0 # in seconds

        # n_to_align = 800 # NB poses to align. In the example data first 600 poses are static. Which is bad.
        # Reference, estimated trajectory, t_max_diff, t_offset. t_max_diff is by default 10ms (0.01)
        traj_reference, traj_estimated = sync.associate_trajectories(traj_reference, traj_estimated, t_max_diff, t_offset)

        # Check length of traj_estimated and traj_ref and get the min
        n_to_align = int(min(traj_reference.num_poses, traj_estimated.num_poses) / 2)

        traj_ref = PoseTrajectory3D(
            positions_xyz=np.stack(traj_reference._positions_xyz, 0),
            orientations_quat_wxyz=np.stack(traj_estimated._orientations_quat_wxyz, 0),
            timestamps=np.array(traj_reference.timestamps))

        est_name = "APE Test #{}".format(test_name)

        result = main_ape.ape(traj_ref, traj_estimated, est_name=est_name,
                                pose_relation=PoseRelation.point_distance, align=True, correct_scale=correct_scale, n_to_align=n_to_align, change_unit=Unit.meters)
        results.append(result)

        print(results)

        fig = plt.figure()
        ax = plot.prepare_axis(fig, plot.PlotMode.xyz)
        # plot.traj(ax, plot.PlotMode.xyz, traj_ref, style="--", alpha=0.5)
        plot.traj_scatter(ax, plot_mode=plot.PlotMode.xyz, traj=traj_ref, marker='+', color='blue', alpha=0.7, label=test_name)

        # plot.traj_colormap(
        #     ax, result.trajectories[est_name], result.np_arrays["error_array"], plot.PlotMode.xyz,
        #     min_map=result.stats["min"], max_map=result.stats["max"], title=results[-1].info["title"]+" " +test_name, plot_start_end_markers=True)

        plot.custom_traj_colormap(
            ax, result.trajectories[est_name], result.np_arrays["error_array"], plot.PlotMode.xyz,
            min_map=result.stats["min"], max_map=result.stats["max"], title=results[-1].info["title"]+" " + test_name, plot_start_end_markers=True, motion_threshold = 1.0)
        

        file_interface.save_res_file(base_path + "/ate_results_" + test_name + ".zip", results[-1],
                                        confirm_overwrite=not True)

        print ("APE " + test_name + "compared_pose_pairs %d pairs"%(len(results[-1].np_arrays["error_array"])))
        print ("rmse %f m"%results[-1].stats['rmse'])
        print ("mean %f m"%results[-1].stats['mean'])
        print ("median %f m"%results[-1].stats['median'])
        print ("std %f m"%results[-1].stats['std'])
        print ("min %f m"%results[-1].stats['min'])
        print ("max %f m"%results[-1].stats['max'])

        plot_x_dimension  = "seconds"
        # Plot the raw metric values.
        fig1 = plt.figure(figsize=SETTINGS.plot_figsize)
        if (plot_x_dimension == "distances"
                and "distances_from_start" in results[-1].np_arrays):
            x_array = results[-1].np_arrays["distances_from_start"]
            x_label = "$d$ (m)"
        elif (plot_x_dimension == "seconds"
                and "seconds_from_start" in results[-1].np_arrays):
            x_array = results[-1].np_arrays["seconds_from_start"]
            x_label = "$t$ (s)"
        else:
            x_array = None
            x_label = "index"

        plot.error_array(
            fig1.gca(), results[-1].np_arrays["error_array"], x_array=x_array,
            statistics={
                s: results[-1].stats[s]
                for s in SETTINGS.plot_statistics if s not in ("min", "max")
            }, name=results[-1].info["label"], title=results[-1].info["title"]+" " +test_name,
            xlabel=x_label, marker="o", linestyle="-")

        plt.tight_layout()
        plt.show()

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="A general point relation alignment script")

    parser.add_argument(
        "--base_path", type=str, default="/home/ttuna/Videos/dlio_verification/",
        help="Base path of the data"
    )

    args = parser.parse_args()

    run_ate(args)