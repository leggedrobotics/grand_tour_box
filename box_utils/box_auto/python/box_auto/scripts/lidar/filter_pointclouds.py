import argparse
import shlex

from box_auto.utils import get_bag, upload_bag, run_ros_command, kill_roscore


BAG_PATTERNS = {
    "livox": "*nuc_livox.bag",
    "hesai": "*nuc_hesai_post_processed.bag",
    "velodyne": "*npc_velodyne_processed.bag",
}


def resolve_inputs(lidars=None):
    """Discover local sensors, or require selected sensors (including downloads)."""
    selected = list(BAG_PATTERNS) if lidars is None else list(dict.fromkeys(lidars))
    unknown = set(selected) - set(BAG_PATTERNS)
    if unknown:
        raise ValueError(f"Unknown LiDAR(s): {', '.join(sorted(unknown))}")
    bags = []
    for lidar in selected:
        bag = get_bag(
            BAG_PATTERNS[lidar],
            auto_download=lidars is not None,
            return_upon_no_files=lidars is None,
        )
        if bag:
            bags.append((lidar, bag))
    if not bags:
        raise FileNotFoundError("No LiDAR input bags found")
    return bags


def main(argv=None):
    parser = argparse.ArgumentParser(description="Filter selected LiDAR bags, or discover local inputs.")
    parser.add_argument("--lidars", nargs="+", help="LiDAR names separated by spaces or commas: livox, hesai, velodyne")
    args = parser.parse_args(argv)
    selected = None if args.lidars is None else [name for item in args.lidars for name in item.split(",")]
    bags = resolve_inputs(selected)
    suffixes = {
        "livox": ("nuc_livox.bag", "nuc_livox_ready.bag"),
        "hesai": ("nuc_hesai_post_processed.bag", "nuc_hesai_ready.bag"),
        "velodyne": ("npc_velodyne_processed.bag", "npc_velodyne_ready.bag"),
    }
    out_bags = []
    kill_roscore()
    try:
        for lidar, bag in bags:
            source_suffix, output_suffix = suffixes[lidar]
            out_bag = bag[:-len(source_suffix)] + output_suffix
            status = run_ros_command(
                "roslaunch box_auto box_filter_lidars.launch "
                f"global_input_bag_path:={shlex.quote(bag)} global_output_bag_path:={shlex.quote(out_bag)}"
            )
            if status != 0:
                raise RuntimeError(f"LiDAR filtering failed for {bag} (exit status {status})")
            out_bags.append(out_bag)
    finally:
        kill_roscore()
    upload_bag(out_bags)


if __name__ == "__main__":
    main()
