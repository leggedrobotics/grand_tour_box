cd /home/jonfrey/git/grand_tour_box/box_utils/box_converter/cluster/.export

SINGULARITY_NOHTTPS=1 singularity build --sandbox grand-tour-dataset.sif docker-daemon://grand-tour-dataset:latest

sudo tar -cvf grand-tour-dataset.tar grand-tour-dataset.sif
scp grand-tour-dataset.tar jonfrey@euler:/cluster/work/rsl/jonfrey/grand_tour/containers

# srun --account=es_hutter --ntasks=1 --cpus-per-task=1 --gpus=1 --time=4:00:00 --mem-per-cpu=8024 --pty bash