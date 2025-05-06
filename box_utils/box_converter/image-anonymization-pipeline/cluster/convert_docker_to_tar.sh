cd /home/jonfrey/git/grand_tour_box/box_utils/box_converter/cluster/.export

apptainer build --sandbox grand-tour-dataset.sif docker-daemon://grand-tour-dataset:latest; sudo tar -cvf grand-tour-dataset.tar grand-tour-dataset.sif; scp grand-tour-dataset.tar jonfrey@euler:/cluster/work/rsl/jonfrey/grand_tour/containers
