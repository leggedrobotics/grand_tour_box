import argparse
from boxi import shell_run
from pathlib import Path

def main(overwrite: bool):
    # Get all .bag.active files
    bag_files = [str(s) for s in Path("/mission_data").rglob("*.bag.active")]
    
    if len(bag_files) == 0:
        print("No active bag files found. --- Finished cleanly!")
        return
    
    for p in bag_files:
        output_file = p.replace('.active', '')
        
        # If overwrite is False, check if the output file already exists
        if not overwrite and Path(output_file).exists():
            print(f"Output file {output_file} already exists. Skipping...")
            continue
        
        # Execute rosbag fix and reindex commands
        # shell_run(f"rosbag fix {p} {output_file}")
        shell_run(f"rosbag reindex {output_file}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Fix and reindex ROS bag files.")
    parser.add_argument(
        "--overwrite",
        type=bool,
        default=True,
        help="Whether to overwrite existing bag files (default: True)."
    )
    args = parser.parse_args()
    
    # Run main function with the parsed argument
    main(args.overwrite)