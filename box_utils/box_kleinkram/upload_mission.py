from pathlib import Path
import os
MISSION_DATA = os.environ.get("MISSION_DATA", "/mission_data")

folder = Path(MISSION_DATA)
files_to_upload = [str(s) for s in folder.rglob("*.bag")]
mission_name = folder.name
project = "GrandTourShakeout"
files_to_upload = ",".join(files_to_upload)
os.system(f"klein upload --path {files_to_upload} --project {project} --mission {mission_name}")
