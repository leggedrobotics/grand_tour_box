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

# temporarily override some package settings
SETTINGS.plot_figsize = [12, 8]
SETTINGS.plot_split = True
SETTINGS.plot_usetex = False

# magic plot configuration
import matplotlib.pyplot as plt


def run_ate(args):  

    # Base path of the data
    base_path = args.base_path
    test_name = args.test_name

    # In the SE3 APE example, you need the reference trajectory and estimated trajectory tracking the same sensor frame.
    # You can't just align 2 random trajectories. For that, we have the point relation example.
    # Here, for example I will use open3d slam as reference
    traj_reference = file_interface.read_tum_trajectory_file(base_path + "slam_poses.txt")
    traj_estimated = file_interface.read_tum_trajectory_file(base_path + "lidar_map_odometry.tum")

    """
    PARAMETERS
    """
    correct_scale = False
    t_max_diff = 0.02 # in seconds 0.02 ->? 20ms
    t_offset = 0.0 # in seconds
    n_to_align = 800 # NB poses to align. In the example data first 600 poses are static. Which is bad.

    # IMPORTANT: full_transformation is unitless. As we mix rotation and translation. (rad + m)
    relation = PoseRelation.translation_part # PoseRelation.full_transformation # PoseRelation.translation_part

    # Reference, estimated trajectory, t_max_diff, t_offset. t_max_diff is by default 10ms (0.01)
    traj_reference, traj_estimated = sync.associate_trajectories(traj_reference, traj_estimated, t_max_diff, t_offset)


    count = 0
    results = []

    est_name="APE Test #{}".format(count)

    
    result = main_ape.ape(traj_reference, traj_estimated, est_name=est_name,
                            pose_relation=relation, 
                            align=True, correct_scale=correct_scale, 
                            n_to_align=n_to_align, change_unit=Unit.meters)
    count += 1
    results.append(result)

    print(results)

    fig = plt.figure()
    ax = plot.prepare_axis(fig, plot.PlotMode.xyz)
    plot.traj(ax, plot.PlotMode.xyz, traj_reference, style="--", alpha=0.5)
    plot.traj_colormap(
        ax, result.trajectories[est_name], result.np_arrays["error_array"], plot.PlotMode.xyz,
        min_map=result.stats["min"], max_map=result.stats["max"])


    file_interface.save_res_file(base_path + "/ape_results" + test_name+ ".zip", results[0],
                                    confirm_overwrite=not True)

    print ("APE compared_pose_pairs %d pairs"%(len(results[0].np_arrays["error_array"])))
    print ("rmse %f m"%results[0].stats['rmse'])
    print ("mean %f m"%results[0].stats['mean'])
    print ("median %f m"%results[0].stats['median'])
    print ("std %f m"%results[0].stats['std'])
    print ("min %f m"%results[0].stats['min'])
    print ("max %f m"%results[0].stats['max'])


    plot_x_dimension  = "seconds"
    # Plot the raw metric values.
    fig1 = plt.figure(figsize=SETTINGS.plot_figsize)
    if (plot_x_dimension == "distances"
            and "distances_from_start" in results[0].np_arrays):
        x_array = results[0].np_arrays["distances_from_start"]
        x_label = "$d$ (m)"
    elif (plot_x_dimension == "seconds"
            and "seconds_from_start" in results[0].np_arrays):
        x_array = results[0].np_arrays["seconds_from_start"]
        x_label = "$t$ (s)"
    else:
        x_array = None
        x_label = "index"


    plot.error_array(
        fig1.gca(), results[0].np_arrays["error_array"], x_array=x_array,
        statistics={
            s: results[0].stats[s]
            for s in SETTINGS.plot_statistics if s not in ("min", "max")
        }, name=results[0].info["label"], title=results[0].info["title"],
        xlabel=x_label)

    plt.tight_layout()    
    plt.show()

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Fix, reindex, and merge ROS bag files.")
    parser.add_argument(
        "--test_name", type=str, default="open3d_slam", help="Name of the test"
    )

    parser.add_argument(
        "--base_path", type=str, default="/home/ttuna/mit_ws/src/open3d_slam_private/open3d_slam_rsl/ros/open3d_slam_ros/data/maps/",
        help="Base path of the data"
    )
    args = parser.parse_args()
    run_ate(args)