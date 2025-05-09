from box_auto.utils import get_uuid_mapping, read_sheet_data
import os
import time


SPREADSHEET_ID = "1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg"
topic_data, MISSION_DATA = read_sheet_data(SPREADSHEET_ID)

SCHEDULE = True
host = "euler.ethz.ch"

submit_dir = os.path.expanduser("~/git/grand_tour_box/box_utils/box_converter/nerfstudio/.submit")
if not os.path.exists(submit_dir):
    os.makedirs(submit_dir)

uuid_mappings = get_uuid_mapping()


for name, data in uuid_mappings.items():
    if MISSION_DATA[name]["GOOD_MISSION"] == "TRUE":
        uuid = data["uuid_release"]
        content = f"""#!/bin/bash

#SBATCH --account=es_hutter
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --time=4:00:00
#SBATCH --mem-per-cpu=4048
#SBATCH --tmp=100000
#SBATCH --output="/cluster/scratch/jonfrey/.submit/{name}_nerfstudio_out.log"
#SBATCH --error="/cluster/scratch/jonfrey/.submit/{name}_nerfstudio_err.log"
#SBATCH --open-mode=truncate


tar -xf /cluster/work/rsl/jonfrey/grand_tour/containers/grand-tour-kleinkram.tar  -C $TMPDIR; mkdir $TMPDIR/grand-tour-kleinkram.sif/out

apptainer exec --nv --writable --bind /cluster/scratch/jonfrey/out:/out --bind /cluster/home/jonfrey/grand_tour_box:/home/catkin_ws/src/grand_tour_box --env KLEINKRAM_CONFIG="$(cat ~/.kleinkram.json)" --containall $TMPDIR/grand-tour-kleinkram.sif /bin/bash -c 'export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH; source /home/opencv_gtsam_ws/devel/setup.bash; source /home/catkin_ws/devel/setup.bash; export HOME=/home; export KLEINKRAM_ACTIVE=ACTIVE; /entrypoint_euler.sh  python3 /home/catkin_ws/src/grand_tour_box/box_utils/box_converter/nerfstudio/nerf_studio.py --mission_uuid {uuid}'


exit 0
        """

        script_path = os.path.join(submit_dir, f"{name}_nerfstudio.sh")
        with open(script_path, "w") as file:
            file.write(content)

os.system(f"scp -r {submit_dir} {host}:/cluster/scratch/jonfrey/")
if SCHEDULE:
    i = 0
    for name, _ in uuid_mappings.items():
        if MISSION_DATA[name]["GOOD_MISSION"] == "TRUE":
            os.system(f"ssh {host} sbatch /cluster/scratch/jonfrey/.submit/{name}_nerfstudio.sh")
            time.sleep(120)
