import os
import glob
from boxi import LOCAL_HOSTNAME, shell_run

def add_arguments(parser):
    parser.set_defaults(main=main)
    parser.add_argument("--pattern", type=str, required=True, help="Glob pattern to match run directories, eg '2024-06-27*'")
    return parser

def main(args):
    # Get the list of directories matching the pattern
    dirs = glob.glob(args.pattern)

    # Get the total number of directories
    total_dirs = len(dirs)

    # Initialize counter
    counter = 1

    for dir in dirs:
        if os.path.isdir(dir):
            # Extract the directory name
            dirname = os.path.basename(dir)
            # Print progress
            print(f"Processing directory {counter} of {total_dirs}: {dirname}")
            cmd = f"rosbags-convert {dirname}/hdr/ --dst ./{dirname}/{dirname}_hdr.bag"
            shell_run(cmd)
            # Increment counter
            counter += 1

    print("All directories processed. <logname>_hdr.bag files created in each directory.")
