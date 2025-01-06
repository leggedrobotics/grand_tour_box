import os
import re
from pathlib import Path


def extract_zip_files(directory):
    """Extracts all zip files in the specified directory using the unzip command."""
    for file in os.listdir(directory):
        if file.endswith(".zip"):
            zip_path = os.path.join(directory, file)
            command = f"unzip -o {zip_path} -d {directory}"  # Overwrite existing files
            os.system(command)
            print(f"Extracted: {file}")


def get_key(s):
    # Find the first numeric sequence in the string
    match = re.search(r"\d+", s)
    if match:
        # Get the end position of the first number
        return s[: match.end()]
    return -1  # Return -1 if no numbers are found


def merge_files_with_teqc(directory):
    """Merges matching .24o files using teqc based on prefixes."""
    # Group files by prefix (e.g., 'dueb326')
    for tag in [".24o", ".24n", ".24l", ".24g", ".24f"]:
        files_to_merge = {}
        for f in [f for f in Path(directory).rglob(f"*{tag}")]:
            k = get_key(f.stem)
            if k not in files_to_merge:
                files_to_merge[k] = [f]
            else:
                files_to_merge[k].append(f)

        for key, files in files_to_merge.items():
            os.makedirs(os.path.join(directory, "out"), exist_ok=True)

            output_file = os.path.join(directory, "out", f"{key}{tag}")
            input_files = " ".join([str(s) for s in files])
            # Use teqc to merge files
            command = f"/home/jonfrey/Desktop/ringo merge {input_files} > {output_file}"

            result = os.system(command)
            if result == 0:
                print(f"Merged {len(files)} files into {output_file}")
            else:
                print(f"Error merging files for {tag} {key}: Command failed with code {result}")


if __name__ == "__main__":
    # Specify the directory
    directory = "/media/jonfrey/T7/ALL_RINEX_DATA"

    # Step 1: Extract all zip files
    extract_zip_files(directory)

    # Step 2: Merge files with teqc
    merge_files_with_teqc(directory)
