#!/bin/bash
# author: Jonas Frey
set -e

# Define usage
__usage="
Usage: $(basename $0) [OPTIONS]
"



# Default target
REMOVE_FLAG="--rm"
INTERACTIVE_FLAG="-it"
IMAGE="leggedrobotics/box"

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


# Run docker #/bin/bash \
   #-eHOST_USERNAME=rsl \
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
    -v /mission_data:/mission_data \
    --gpus all \
    $IMAGE \
    $COMMAND