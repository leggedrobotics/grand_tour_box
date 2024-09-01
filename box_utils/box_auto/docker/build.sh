# Adapt the ssh-key and the paths
export DOCKER_BUILDKIT=1
docker build --progress=plain --ssh default=/home/jonfrey/.ssh/id_rsa_2 -t leggedrobotics/box -f /home/jonfrey/git/grand_tour_box/box_utils/box_auto/docker/box.Dockerfile "/home/jonfrey/git/grand_tour_box/box_utils/box_auto/docker"