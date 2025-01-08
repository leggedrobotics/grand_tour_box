from box_auto.utils import MISSION_DATA, BOX_AUTO_SCRIPTS_DIR
from pathlib import Path
import os

subfolders = ["extrinsic_offset", "time_offset"]
NEW_MISSION_DATA = []
for s in subfolders:
    NEW_MISSION_DATA += [str(f.parent) for f in (Path(MISSION_DATA) / s).rglob("*_nuc_hesai_post_processed.bag")]

for m in NEW_MISSION_DATA:
    os.system(f"export MISSION_DATA={m}; python {BOX_AUTO_SCRIPTS_DIR}/application/dlio.py")
    dlio_out_bag = [str(p) for p in Path(m).glob("*dlio.bag")][0]
    tag = m.split("/")[-1]
    dlio_renamed_bag = dlio_out_bag.replace("dlio.bag", f"dlio_{tag}.bag")
    os.system(f"cp {dlio_out_bag} {dlio_renamed_bag}")
