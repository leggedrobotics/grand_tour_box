#!/usr/bin/env bash
set -euo pipefail

if [[ $# != 1 || ! -d "$1/src" || ! -d "$1/.catkin_tools" ]]; then
    echo "Usage: $0 /absolute/path/to/existing/catkin_workspace" >&2
    echo "Source the workspace's ROS environment first." >&2
    exit 2
fi
workspace=$(realpath "$1")
source_dir="$workspace/src/rviz_plugin_tutorials_compat"
# Same upstream release as the Noetic binary plugin. Rebuild against the RViz
# headers/libraries selected by this workspace instead of mixing their ABIs.
revision=3141f0dfcb7c34ce57909226d5394ca5fd57a5c7
repository=https://github.com/ros-visualization/visualization_tutorials.git

if [[ -e "$source_dir" ]]; then
    if [[ ! -d "$source_dir/.git" ]] ||
       [[ $(git -C "$source_dir" rev-parse HEAD) != "$revision" ]] ||
       [[ -n $(git -C "$source_dir" status --porcelain) ]]; then
        echo "Existing $source_dir differs from the pinned source; preserving it." >&2
        exit 1
    fi
else
    git clone --filter=blob:none --no-checkout "$repository" "$source_dir"
    git -C "$source_dir" sparse-checkout init --cone
    git -C "$source_dir" sparse-checkout set rviz_plugin_tutorials
    git -C "$source_dir" checkout --detach "$revision"
fi

# A workspace build makes the repaired plugin discoverable after sourcing the
# normal devel/setup.bash. System ROS packages and unrelated sources stay intact.
catkin build --workspace "$workspace" rviz_plugin_tutorials --no-deps --no-status --force-cmake -j 2
echo "Plugin rebuilt. Source $workspace/devel/setup.bash before launching RViz."
