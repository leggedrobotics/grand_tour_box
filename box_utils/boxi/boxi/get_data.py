from boxi import shell_run


def add_arguments(parser):
    parser.set_defaults(main=main)
    parser.add_argument("--jetson", action="store_true", help="Get data from Jetson")
    parser.add_argument("--nuc", action="store_true", help="Get data from Nuc")
    parser.add_argument("--lpc", action="store_true", help="Get data from Nuc")
    parser.add_argument("--npc", action="store_true", help="Get data from Nuc")
    parser.add_argument("--check", action="store_true", help="Get data from Nuc")
    parser.add_argument("--post-only", action="store_true", help="Get data from Nuc")
    parser.add_argument(
        "--directory", type=str, nargs="+", required=True, help="One or more specific directories inside /data to sync"
    )
    return parser


def main(args):
    rsync_exclusions = ""
    hosts = []
    users = []
    if args.jetson:
        hosts.append("jetson")
        users.append("rsl")
        rsync_exclusions = '--exclude="workspaces"'
        rsync_exclusions += ' --exclude="docker"'
    if args.nuc:
        hosts.append("nuc")
        users.append("rsl")
    if args.lpc:
        hosts.append("lpc")
        users.append("rsl")
    if args.npc:
        hosts.append("npc")
        users.append("rsl")

    for directory in args.directory:
        if directory:
            if args.post_only:
                keys = [
                    "_jetson_utils.bag",
                    "_jetson_stim.bag",
                    "_jetson_ap20_aux.bag",
                    "_jetson_adis.bag",
                    "_jetson_zed2i_tf.bag",
                    "_jetson_zed2i_prop.bag",
                    "_jetson_zed2i_images.bag",
                    "_jetson_zed2i_depth.bag",
                    "_jetson_hdr_right_updated.bag",
                    "_jetson_hdr_front_updated.bag",
                    "_jetson_hdr_left_updated.bag",
                    "_tf_static.bag",
                    "_nuc_hesai_post_processed.bag",
                    "_nuc_utils.bag",
                    "_nuc_tf.bag",
                    "_nuc_livox.bag",
                    "_nuc_hesai.bag",
                    "_nuc_cpt7.bag",
                    "_nuc_alphasense.bag",
                    "_npc_depth_cameras.bag",
                    "_npc_elevation_mapping.bag",
                    "_npc_velodyne.bag",
                    "_lpc_anymal_imu.bag",
                    "_lpc_depth_cameras.bag",
                    "_lpc_general.bag",
                    "_lpc_locomotion.bag",
                    "_lpc_state_estimator.bag",
                    "_lpc_tf.bag",
                    ".log",
                    ".svo",
                    ".yaml",
                ]
                include_patterns = " ".join([f"--include='*{key}'" for key in keys])
                rsync_part2 = f":/data/{directory}* . --include=*/ {include_patterns} --exclude='*' "
            else:
                rsync_part2 = f":/data/{directory} . {rsync_exclusions}"

            if len(hosts) == 0:
                print("No host specified. Specify host with --hostname")
            for host, user in zip(hosts, users):
                if args.check:
                    # We want maximum speed of 1.5 Gbit/s -> 1.5 * 10 **6 / 8 = 187500 KB/s -> --bwlimit 187500
                    rsync_part1 = "rsync -r --progress -Pv --size-only -n " + user + "@"
                else:
                    rsync_part1 = "rsync -r --progress -Pv --size-only " + user + "@"

                cmd = f"{rsync_part1}{host}{rsync_part2}"
                print(cmd)
                shell_run(cmd)
