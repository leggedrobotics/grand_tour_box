from box_auto.utils import get_uuid_mapping
import os

SCHEDULE = True
cameras = ["hdr", "alphasense", "zed2i"]
host = "euler.ethz.ch"
info_scratch = {"hdr": 80000, "alphasense": 80000, "zed2i": 200000}
nr_of_missions = 1

submit_dir = os.path.expanduser("~/git/grand_tour_box/box_utils/box_converter/cluster/.submit")
if not os.path.exists(submit_dir):
    os.makedirs(submit_dir)

uuid_mappings = get_uuid_mapping()

for name, data in uuid_mappings.items():
    uuid = data["uuid_pub"]
    for camera in cameras:
        scratch = info_scratch[camera]

        content = f"""#!/bin/bash

#SBATCH --account=es_hutter
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --gpus=1
#SBATCH --time=4:00:00
#SBATCH --mem-per-cpu=4048
#SBATCH --tmp={scratch}
#SBATCH --output="/cluster/scratch/jonfrey/.submit/{name}_{camera}_out.log"
#SBATCH --error="/cluster/scratch/jonfrey/.submit/{name}_{camera}_err.log"
#SBATCH --open-mode=truncate


tar -xf /cluster/work/rsl/jonfrey/grand_tour/containers/grand-tour-dataset.tar  -C $TMPDIR

module load stack/2024-04 gcc/8.5.0 cuda/12.1.1 eth_proxy

apptainer exec --nv --containall --writable --env KLEINKRAM_CONFIG="$(cat ~/.kleinkram.json)" $TMPDIR/grand-tour-dataset.sif /bin/bash -c 'export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH; export HOME=/home; /entrypoint.sh  python /app/anonymization.py --mission-id {uuid} --cam {camera}'

exit 0
        """

        script_path = os.path.join(submit_dir, f"{name}_{camera}.sh")
        with open(script_path, "w") as file:
            file.write(content)


if SCHEDULE:
    os.system(f"scp -r {submit_dir} {host}:/cluster/scratch/jonfrey/")
    for j, name in enumerate(uuid_mappings.keys()):
        if j >= nr_of_missions:
            break

        for camera in cameras:
            os.system(f"ssh {host} sbatch /cluster/scratch/jonfrey/.submit/{name}_{camera}.sh")
