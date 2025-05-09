from box_auto.utils import upload_video
from pathlib import Path
import yaml

uploaded_videos_file = Path("/home/jonfrey/Videos/video_ids_fist_upload_run.yaml")
with open(uploaded_videos_file, "r") as f:
    uploaded_videos = yaml.safe_load(f)

uploaded_videos = [list(k.keys())[0] for k in uploaded_videos]
res = {}
for p in Path("/home/jonfrey/Videos").glob("*.mp4"):
    if p.stem not in uploaded_videos:
        video_id = upload_video(file_path=str(p), title=f"GrandTour Preview {p.stem}")
        res[p.stem] = video_id

with open("/home/jonfrey/Videos/video_ids.txt", "w") as f:
    yaml.dump(res, f)
