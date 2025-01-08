#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import os
import yaml
from evo.core.trajectory import PoseTrajectory3D
from evo.core import sync
from evo.tools import plot, file_interface, log
import evo.main_ape as main_ape
import evo.main_rpe as main_rpe
from evo.core.metrics import PoseRelation
from evo.core.metrics import Unit
import evo.common_ape_rpe as common
from evo.tools.settings import SETTINGS
import matplotlib.cm as cm
import matplotlib as mpl


SETTINGS.plot_figsize = [12, 8]
SETTINGS.plot_split = True
SETTINGS.plot_usetex = False
SETTINGS.plot_xyz_realistic = False
SETTINGS.plot_show_legend = True
SETTINGS.plot_show_axis = True

# magic plot configuration
import matplotlib.pyplot as plt


def run_ate(test_name, reference_file, estimated_file, params, output_path, disable_viz, relation):

    traj_reference = file_interface.read_tum_trajectory_file(reference_file)
    traj_estimated = file_interface.read_tum_trajectory_file(estimated_file)

    t_max_diff = params.get("t_max_diff", 0.02)
    t_offset = params.get("t_offset", 0.0)
    n_to_align = params.get("n_to_align", 800)

    if n_to_align == -1:
        alignment_post_fix = "all"
    else:
        alignment_post_fix = str(n_to_align)

    if relation == "translation_part":
        relation = PoseRelation.translation_part
        relation_post_fix = "tp"
        print("\033[91translation_part.\033[0m")
    elif (relation == "rotation_angle_deg"):
        relation = PoseRelation.rotation_angle_deg
        relation_post_fix = "angle_deg"
        print("\033[91Angle deg.\033[0m")
    else:
        print("\033[91mInvalid relation.\033[0m")
        return

    correct_scale = params.get("correct_scale", False)

    test_name +=  "_" + relation_post_fix + "_" + alignment_post_fix

    # Reference, estimated trajectory, t_max_diff, t_offset. t_max_diff is by default 10ms (0.01)
    
    traj_reference, traj_estimated = sync.associate_trajectories(traj_reference, traj_estimated, t_max_diff, t_offset)

    results = []
    result = main_ape.ape(traj_reference, traj_estimated, est_name=test_name, ref_name="IE",
                            pose_relation=relation, 
                            align=True,
                            align_origin=False,
                            n_to_align=n_to_align,
                            correct_scale=correct_scale
                            )
    results.append(result)
    print(result)

    fig = plt.figure(figsize=(16, 12))

    fig.suptitle("Absolute Pose Error (APE) Trajectory", fontsize=25)
    ax = plot.prepare_axis(fig, plot.PlotMode.xyz)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")

    plot.traj(ax, plot.PlotMode.xyz, traj_reference, style="--", alpha=0.5, label= "Ref.")
    plot.traj(ax, plot.PlotMode.xyz, traj_estimated, style="-", alpha=0.5, label= " "  + test_name + " ")

    ax.view_init(elev=30, azim=45)

    min_map=result.stats["min"]
    max_map=result.stats["max"]
    pos = traj_estimated.positions_xyz
    norm = mpl.colors.Normalize(vmin=min_map, vmax=max_map, clip=True)
    mapper = cm.ScalarMappable(
        norm=norm,
        cmap=SETTINGS.plot_trajectory_cmap)
    array = result.np_arrays["error_array"]
    mapper.set_array(array)
    colors = [mapper.to_rgba(a) for a in array]
    line_collection = plot.colored_line_collection(pos, colors, plot.PlotMode.xyz)
    ax.add_collection(line_collection)
    ax.autoscale_view(True, True, True)
    plot.set_aspect_equal(ax)


    cbar = fig.colorbar(
    mapper, ticks=[min_map, (max_map - (max_map - min_map) / 2), max_map],
    ax=ax)
    cbar.ax.set_yticklabels([
        "{0:0.3f}".format(min_map),
        "{0:0.3f}".format(max_map - (max_map - min_map) / 2),
        "{0:0.3f}".format(max_map)
    ])

    if relation == PoseRelation.translation_part:
        cbar.set_label('APE [m]', labelpad=20)
    elif relation == PoseRelation.rotation_angle_deg:
        cbar.set_label('APE [deg]', labelpad=20)

    cbar.ax.yaxis.label.set_size(25)
    cbar.ax.tick_params(labelsize=25) 
    ax.legend(frameon=True)

    fig.savefig(output_path + "/ape_3d_plot_" + test_name + ".png", dpi=600)

    file_interface.save_res_file(output_path + "/ape_results_" + test_name+ ".zip", results[0],
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
    fig_stats = plt.figure(figsize=SETTINGS.plot_figsize)
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
        fig_stats.gca(), results[0].np_arrays["error_array"], x_array=x_array,
        statistics={
            s: results[0].stats[s]
            for s in SETTINGS.plot_statistics if s not in ("min", "max")
        }, name=results[0].info["label"], title=results[0].info["title"],
        xlabel=x_label)
    
    fig_stats.savefig(output_path + "/ape_statistics_" + test_name + ".png", dpi=600)

    
    if disable_viz == True:
        return
    else:
        plt.tight_layout()
        plt.show()

def process_directory(base_path, output_dir, config, disable_viz, prefix):
    
    # Base path
    if not base_path.endswith('/'):
        base_path += '/'

    # Per test params
    for test_name, test_params in list(config.items()):
        if isinstance(test_params, (bool, str)):
            continue
        estimated_file = test_params.get("estimated_file")
        reference_file = test_params.get("reference_file")
        
        if estimated_file.startswith("_"):
            estimated_file = prefix + estimated_file
        if reference_file.startswith("_"):
            reference_file = prefix + reference_file

        params = test_params.get("parameters", {})
        print(f"\033[92mProcessing test: {test_name}\033[0m")
        print(f"\033[92mEstimated file: {estimated_file}, Reference file: {reference_file}\033[0m")
        run_ate(test_name, base_path + reference_file, base_path + estimated_file, params, output_dir, disable_viz, "translation_part")
        run_ate(test_name, base_path + reference_file, base_path + estimated_file, params, output_dir, disable_viz, "rotation_angle_deg")

def load_config(config_path):
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
    return config

if __name__ == "__main__":

    import argparse

    parser = argparse.ArgumentParser(description="Run APE evaluation for multiple tests.")
    
    # Define command-line arguments
    parser.add_argument("--config", type=str, required=True, help="Path to the configuration YAML file")
    parser.add_argument("--input_folder_path", type=str, required=True,  default="mission_data/", help="Mission data subdirectory (default: 'mission_data/')")
    parser.add_argument("--output_dir_name", type=str, required=True, default="results/", help="Directory to save output plots and results (default: 'results/')")
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
    