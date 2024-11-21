from pathlib import Path
import os

MISSION_DATA = os.environ.get("MISSION_DATA", "/mission_data")

folder = Path(MISSION_DATA)
files_to_upload = [str(s) for s in folder.rglob("*.bag")]
keys = [
    ".bag",
]
upload = []
for f in files_to_upload:
    for k in keys:
        if f.find(k) != -1:
            upload.append(f)

upload.sort()

mission_name = folder.name
project = "GrandTourCalibration"
files_to_upload = " --path " + " --path ".join(files_to_upload)
os.system(f"klein upload --ignore-missing-tags --project {project} --mission {mission_name} --create {files_to_upload}")
