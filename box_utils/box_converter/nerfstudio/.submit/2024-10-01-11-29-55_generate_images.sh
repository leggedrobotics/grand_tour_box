#!/bin/bash

#SBATCH --account=es_hutter
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --gpus=1
#SBATCH --time=10:00:00
#SBATCH --mem-per-cpu=4048
#SBATCH --tmp=100000
#SBATCH --output="/cluster/scratch/jonfrey/.submit/2024-10-01-11-29-55_generate_images_out.log"
#SBATCH --error="/cluster/scratch/jonfrey/.submit/2024-10-01-11-29-55_generate_images_err.log"
#SBATCH --open-mode=truncate


tar -xf /cluster/work/rsl/jonfrey/grand_tour/containers/grand-tour-kleinkram.tar  -C $TMPDIR
module purge
module load stack/2024-04 gcc/8.5.0 cuda/12.1.1

apptainer exec --nv --containall --writable --bind /cluster/scratch/jonfrey:/out --env KLEINKRAM_CONFIG="$(cat ~/.kleinkram.json)" $TMPDIR/grand-tour-kleinkram.sif /bin/bash -c 'export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH; export HOME=/home; /entrypoint_kleinkram.sh  python /app/anonymization.py --mission-id d7525079-5564-461e-a144-e7479247d268 --cam alphasense'

exit 0
        