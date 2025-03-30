import kleinkram
import yaml
from box_auto.utils import BOX_AUTO_DIR
import os

missions = kleinkram.list_missions(project_ids=["3c97a27e-4180-4e40-b8af-59714de54a87"], mission_names=["*"])

missions_pub = [m for m in missions if m.name.startswith("pub_")]
missions_raw = [m for m in missions if not m.name.startswith("pub_")]

UUIDS = {}
for mr in missions_raw:
    for mp in missions_pub:
        if mr.name == mp.name[4:]:
            UUIDS[mr.name] = {"uuid": str(mr.id), "uuid_pub": str(mp.id)}
            print(f"Matched: {mr.name} with {mp.name}")
            break


with open(os.path.join(BOX_AUTO_DIR, "cfg", "klein_uuid_mapping.yaml"), "w") as f:
    yaml.dump(UUIDS, f)
