from box_auto.utils import get_uuid_mapping, read_sheet_data
import os
import time


SPREADSHEET_ID = "1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg"
topic_data, MISSION_DATA = read_sheet_data(SPREADSHEET_ID)

SCHEDULE = True
cameras = ["zed2i", "hdr", "alphasense"]
host = "euler.ethz.ch"
info_scratch = {"hdr": 100000, "alphasense": 100000, "zed2i": 200000}
info_time = {"hdr": "10:00:00", "alphasense": "10:00:00", "zed2i": "8:00:00"}
nr_of_missions = 100

submit_dir = os.path.expanduser("~/git/grand_tour_box/box_utils/box_converter/cluster/.submit")
if not os.path.exists(submit_dir):
    os.makedirs(submit_dir)

uuid_mappings = get_uuid_mapping()


for name, data in uuid_mappings.items():
    if MISSION_DATA[name]["GOOD_MISSION"] == "TRUE":
        uuid = data["uuid_pub"]
        for camera in cameras:
            scratch = info_scratch[camera]
            ts = info_time[camera]
            content = f"""#!/bin/bash

#SBATCH --account=es_hutter
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --gpus=1
#SBATCH --time={ts}
#SBATCH --mem-per-cpu=4048
#SBATCH --tmp={scratch}
#SBATCH --output="/cluster/scratch/jonfrey/.submit/{name}_{camera}_out.log"
#SBATCH --error="/cluster/scratch/jonfrey/.submit/{name}_{camera}_err.log"
#SBATCH --open-mode=truncate


tar -xf /cluster/work/rsl/jonfrey/grand_tour/containers/grand-tour-dataset.tar  -C $TMPDIR
module purge
module load stack/2024-04 gcc/8.5.0 cuda/12.1.1

apptainer exec --nv --containall --writable --bind /cluster/scratch/jonfrey:/scratch --env KLEINKRAM_CONFIG="$(cat ~/.kleinkram.json)" $TMPDIR/grand-tour-dataset.sif /bin/bash -c 'export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH; export HOME=/home; /entrypoint.sh  python /app/anonymization.py --mission-id {uuid} --cam {camera}'

exit 0
        """

            script_path = os.path.join(submit_dir, f"{name}_{camera}.sh")
            with open(script_path, "w") as file:
                file.write(content)

os.system(f"scp -r {submit_dir} {host}:/cluster/scratch/jonfrey/")
if SCHEDULE:
    for name, _ in uuid_mappings.items():
        if MISSION_DATA[name]["GOOD_MISSION"] == "TRUE":
            for camera in cameras:
                os.system(f"ssh {host} sbatch /cluster/scratch/jonfrey/.submit/{name}_{camera}.sh")
                time.sleep(60 * 5)
