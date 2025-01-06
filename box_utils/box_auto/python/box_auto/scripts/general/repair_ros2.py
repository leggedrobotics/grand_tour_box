import argparse
from pathlib import Path
import os
from box_auto.utils import MISSION_DATA


def replace_strings_in_yaml(file_path, new_relative_file_path, out_file, new_hdr_camera):
    # Open the output file for writing
    with open(out_file, "w") as ofile:
        # Open the input YAML file for reading
        with open(file_path, "r+") as file:
            # Read and process each line
            for line in file:
                # Replace the target string with the new relative file path
                modified_line = line.replace("replace_me_with_relative_file_path", new_relative_file_path)
                modified_line = modified_line.replace("replace_hdr_camera", new_hdr_camera)

                # Write the modified line to the output file
                ofile.write(modified_line)


def main(overwrite: bool):
    # Get all .bag.active files
    bag_files = [s for s in Path(MISSION_DATA).rglob("*.mcap")]

    for p in bag_files:
        metadata = p.parent / "metadata.yaml"
        if metadata.exists():
            print(f"Metadata file {metadata} found. Skipping...")
            continue

        # Create new metadata.yaml file (hacky)

        camera = "_".join(p.stem.split("_")[2:4])
        replace_strings_in_yaml(
            "/home/catkin_ws/src/grand_tour_box/box_utils/box_auto/cfg/metadata.yaml",
            p.stem + ".mcap",
            p.parent / "metadata.yaml",
            camera,
        )

        if str(p).find(".mcap.active") == -1:
            active_file = str(p.with_suffix(p.suffix + ".active"))
            os.system(f"mv {p} {active_file}")
            output = str(p)
        else:
            active_file = str(p)
            output = str(p).replace(".mcap.active", ".mcap")

        # Then recover the .mcap file and reindex it
        cmd = f"/bin/mcap recover -o={output} {active_file}"
        print(cmd)
        os.system(cmd)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Fix and reindex ROS bag files.")
    parser.add_argument(
        "--overwrite", type=bool, default=True, help="Whether to overwrite existing bag files (default: True)."
    )
    args = parser.parse_args()

    print("Only works with compressed images given that metadata.yaml is hardcoded to use the compressed image topic.")
    # Run main function with the parsed argument
    main(args.overwrite)

    print("Only works with compressed images given that metadata.yaml is hardcoded to use the compressed image topic.")
