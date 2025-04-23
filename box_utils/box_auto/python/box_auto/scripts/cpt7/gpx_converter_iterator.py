import os
import subprocess
from pathlib import Path
from box_auto.utils import WS

# Set the root directory to search (use '.' for current directory)
root_dir = Path("<enter-your-path>").resolve()

# Find all *_tc_statistics.json files recursively
json_files = list(root_dir.rglob("*_tc_statistics.json"))

# Deduplicate parent directories
parent_dirs = {file.parent.resolve() for file in json_files}

# Path to your Python script
python_script = Path(
    f"{WS}/src/grand_tour_box/box_utils/box_auto/python/box_auto/scripts/cpt7/gpx_converter.py"
).resolve()

# Run processing
for parent_dir in parent_dirs:
    env = os.environ.copy()
    env["MISSION_DATA"] = str(parent_dir)
    subprocess.run(["python3", str(python_script)], env=env)
