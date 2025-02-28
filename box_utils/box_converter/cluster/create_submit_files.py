from box_auto.utils import UUID_MAPPING
import os

SCHEDULE = False
submit_dir = os.path.expanduser("~/git/grand_tour_box/box_utils/box_converter/cluster/.submit")

if not os.path.exists(submit_dir):
    os.makedirs(submit_dir)

for uuid, new_uuid in UUID_MAPPING.items():
    for camera in ["hdr"]:
        content = f"""#!/bin/bash

#SBATCH --account=es_hutter
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=10
#SBATCH --gpus=1
#SBATCH --time=4:00:00
#SBATCH --mem-per-cpu=2048
#SBATCH --tmp=100000
#SBATCH --output="~/res/uuid.log"
#SBATCH --error="~/res/err.log"
#SBATCH --open-mode=truncate


tar -xf /cluster/work/rsl/jonfrey/grand_tour/containers/grand-tour-dataset.tar  -C $TMPDIR

singularity exec --env KLEINKRAM_CONFIG="$(cat ~/.kleinkram.json)" --nv --writable  --containall $TMPDIR/grand-tour-dataset.sif /entrypoint.sh  python /app/anonymization.py --mission-id {uuid} --cam {camera}

exit 0
        """

        script_path = os.path.join(submit_dir, f"{uuid}_{camera}.sh")
        with open(script_path, "w") as file:
            file.write(content)

        if SCHEDULE:
            os.system(f"sbatch {script_path}")
