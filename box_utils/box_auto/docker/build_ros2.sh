# Adapt the ssh-key and the paths
export DOCKER_BUILDKIT=1
docker build --progress=plain --ssh default=$HOME/.ssh/id_rsa_2 -t  rslethz/grand_tour_box_ros2 -f $HOME/git/grand_tour_box/box_utils/box_auto/docker/box_ros2.Dockerfile "$HOME/git/grand_tour_box/box_utils/box_auto/docker"