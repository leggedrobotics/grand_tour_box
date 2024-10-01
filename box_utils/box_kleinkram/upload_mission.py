from pathlib import Path
import os

folder = Path("/data/2024-09-23-10-22-43")
files_to_upload = [str(s) for s in folder.rglob("*.bag")] + [str(s) for s in folder.rglob("*.mcap")]
mission_name = folder.name
project = "GrandTourShakeout"
files_to_upload = ",".join(files_to_upload)
os.system(f"klein upload --path {files_to_upload} --project {project} --mission {mission_name}")
