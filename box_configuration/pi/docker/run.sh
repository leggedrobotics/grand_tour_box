#!/bin/bash
# author: Jonas Frey, Matias Mattamala
set -e

# Define usage
__usage="
Usage: $(basename $0) [OPTIONS]
"

# Default target
REMOVE_FLAG="--rm"
INTERACTIVE_FLAG="-it"
IMAGE="rslethz:noetic-pi-focal"
ENTRYPOINT="/home/rsl/git/grand_tour_box/box_configuration/pi/docker/entrypoint.sh"

COMMAND=""
# Read arguments
for i in "$@"; do
    case $i in
        --no-rm)
            REMOVE_FLAG=""
            shift
            ;;
        -i=*|--image=*)
            IMAGE=${i#*=}
            shift
            ;;
        --no-it)
            INTERACTIVE_FLAG=""
            shift
            ;;
        --command=*)
            COMMAND=${i#*=}
            shift
            ;;
        *)
            echo "$__usage"
            exit 0
            ;;
    esac
done

# Enable graphical stuff launch in the container
# Reference: http://wiki.ros.org/docker/Tutorials/GUI
XSOCK=$HOME/.X11-unix
XAUTH=$HOME/.docker.xauth
if [ ! -f $XAUTH ]; then
    > $XAUTH # make an empty file
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
    chmod a+r $XAUTH
fi

cd ~
mkdir -p .etc && cd .etc
ln -sf /etc/passwd .
ln -sf /etc/shadow .
ln -sf /etc/group .
cd ..


# Run docker
docker run --net=host \
   --privileged \
    $INTERACTIVE_FLAG \
    $REMOVE_FLAG \
    --volume=$XSOCK:/root/.X11-unix:rw \
    --volume=$XAUTH:/root/.docker.xauth:rw \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=/root/.docker.xauth" \
    --env="DISPLAY=$DISPLAY" \
    --ulimit rtprio=99 \
    --cap-add=sys_nice \
    -v$HOME:$HOME \
    -v /sys:/sys \
    -v /dev/gpiochip0:/dev/gpiochip0 \
    -v /dev/gpiochip1:/dev/gpiochip1 \
    --entrypoint=$ENTRYPOINT \
    $IMAGE \
    $COMMAND

   # -eHOST_USERNAME=rsl \
    #-v$(pwd)/.etc/shadow:/etc/shadow \
    #-v$(pwd)/.etc/passwd:/etc/passwd \
    #-v$(pwd)/.etc/group:/etc/group \
