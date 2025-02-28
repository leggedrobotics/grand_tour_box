from box_auto.utils import UUID_MAPPING
import os

# srun --account=es_hutter --ntasks=1 --cpus-per-task=1 --gpus=1 --time=4:00:00 --mem-per-cpu=8024 --pty bash

SCHEDULE = False
submit_dir = os.path.expanduser("~/git/grand_tour_box/box_utils/box_converter/cluster/.submit")

if not os.path.exists(submit_dir):
    os.makedirs(submit_dir)

for uuid, new_uuid in UUID_MAPPING.items():
    for topic_key in ["hdr_left", "hdr_right", "hdr_front"]:
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


module load gcc/8.5.0 cuda/12.1.1
tar -xf /cluster/work/rsl/jonfrey/grand_tour/containers/grand-tour-dataset.tar  -C $TMPDIR
mkdir $TMPDIR/tmp_disk


singularity exec -B /cluster/home/jonfrey/kleinkram.json:/home/ -B $TMPDIR/tmp_disk:/tmp_disk --nv --writable --containall $TMPDIR/grand-tour-dataset.sif /home/run.sh --uuid {uuid} --image {topic_key}
echo "run.sh done"
exit 0
        """

        script_path = os.path.join(submit_dir, f"{uuid}_{topic_key}.sh")
        with open(script_path, "w") as file:
            file.write(content)

        if SCHEDULE:
            os.system(f"sbatch {script_path}")
