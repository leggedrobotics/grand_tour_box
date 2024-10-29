#!/usr/bin/env bash

folder="./"

# Function to display usage
usage() {
    echo "Usage: $0 [--folder <path>]"
    echo "  --folder: Specify the folder path to search for bag files (default: current folder )"
    exit 1
}

args=""
while [[ $# -gt 0 ]]; do
    case $1 in
        --folder)
            folder="$2"
            shift 2
            ;;
        -h|--help)
            usage
            ;;
        *)
            args="$args $1"
            shift 1
            ;;
    esac
done

# Array of keys to filter
keys=(
    # "_lpc_"
    # "_npc_"
    # "_nuc_hesai_post_processed.bag"
    # "_jetson_ap20_synced.bag"
    "_cpt7_raw_imu.bag"
    # "_cpt7_gps_optimized_trajectory.bag"
    "_tf_static.bag"
    # "_nuc_utils.bag"
    # "_nuc_tf.bag"
    "_nuc_livox.bag"
    # "_nuc_hesai.bag"
    # "_nuc_cpt7.bag"
    # "_nuc_alphasense.bag"
    # "_jetson_utils.bag"
    # "_jetson_stim.bag"
    # "_jetson_ap20_aux.bag"
    # "_jetson_adis.bag"
    "_jetson_zed2i_tf.bag"
    "_jetson_zed2i_prop.bag"
    # "_jetson_zed2i_images.bag"
    #"_jetson_zed2i_depth.bag"
    "_jetson_hdr_right.bag"
    "_jetson_hdr_left.bag"
    "_jetson_hdr_front.bag"
    #"_jetson_hdr_front_rect.bag"
)

# Find all .bag files and filter them
filtered_files=()
while IFS= read -r file; do
    for key in "${keys[@]}"; do
        if [[ "$file" == *"$key"* ]]; then
            filtered_files+=("$file")
            break
        fi
    done
done < <(find "$folder" -type f -name "*.bag")

# If no files found, exit
if [ ${#filtered_files[@]} -eq 0 ]; then
    echo "No matching bag files found in $folder"
    exit 1
fi

# Set ROS parameter for simulation time
rosparam set use_sim_time 1

# Play the filtered bag files
rosbag play --clock "${filtered_files[@]}" $args
