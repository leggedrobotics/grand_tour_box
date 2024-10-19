# Adapt the ssh-key and the paths
export DOCKER_BUILDKIT=1
docker build --no-cache --progress=plain --ssh default=$HOME/.ssh/id_rsa_2 -t  rslethz/grand_tour_box -f $HOME/git/grand_tour_box/box_utils/box_auto/docker/box_kleinkram.Dockerfile "$HOME/git/grand_tour_box/box_utils/box_auto/docker"