#!/bin/bash
# author: Jonas Frey
MISSION_DATA_MOUNT="--volume $MISSION_DATA:/mission_data"

set -e

# Define usage
__usage="
Usage: $(basename $0) [OPTIONS]

Required:
  --type=TYPE               Select image type: kleinkram, kleinkram_minimal, ros2, full

Optional:
  --no-rm                  Don't remove container after exit
  --image=NAME             Override default image name (default: rslethz/grand_tour_box)
  --no-it                  Don't run in interactive mode
  --command=CMD            Run specific command in container
  --mission-data=PATH      Mount mission data directory to /mission_data
  --debug                  Mount grand_tour_box repo for development
  --env
"

# Default values
REMOVE_FLAG="--rm"
INTERACTIVE_FLAG="-it"
IMAGE="rslethz/grand_tour_box"
IMAGE_TYPE=""
COMMAND=""
MISSION_DATA=""
DEBUG_MOUNT=""
REPO_PATH="$HOME/git/grand_tour_box"  # Default repo path

ENV_VARS=()


# Read arguments
for i in "$@"; do
    case $i in
        --type=*)
            TYPE=${i#*=}
            case $TYPE in
                kleinkram|kleinkram_minimal|ros2|bridge)
                    IMAGE_TYPE=$TYPE
                    ;;
                *)
                    echo "Error: Invalid type. Must be one of: kleinkram, kleinkram_minimal, ros2, bridge"
                    echo "$__usage"
                    exit 1
                    ;;
            esac
            shift
            ;;
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
        --mission-data=*)
            MISSION_DATA=${i#*=}
            if [ -d "$MISSION_DATA" ]; then
                MISSION_DATA_MOUNT="--volume $MISSION_DATA:/mission_data"
            else
                echo "Error: Mission data path does not exist: $MISSION_PATH"
                exit 1
            fi
            shift
            ;;
        --debug)
            if [ -d "$REPO_PATH" ]; then
                DEBUG_MOUNT="--volume $REPO_PATH:/home/catkin_ws/src/grand_tour_box --net=host"
            else
                echo "Error: Repository path does not exist: $REPO_PATH"
                exit 1
            fi
            shift
            ;;
        --env=*)
            ENV_VARS+=("--env=${i#*=}")
            shift
            ;;
        *)
            echo "$__usage"
            exit 0
            ;;
    esac
done

# Check if type is specified
if [ -z "$IMAGE_TYPE" ]; then
    echo "Error: --type must be specified"
    echo "$__usage"
    exit 1
fi

# Construct full image name with tag
FULL_IMAGE="$IMAGE:$IMAGE_TYPE"

# Enable graphical stuff launch in the container
# Reference: http://wiki.ros.org/docker/Tutorials/GUI
XSOCK=$HOME/.X11-unix
XAUTH=$HOME/.docker.xauth
if [ ! -f $XAUTH ]; then
    > $XAUTH # make an empty file
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
    chmod a+r $XAUTH
fi

# Print configuration
echo "Running docker with:"
echo "  Image: $FULL_IMAGE"
echo "  Remove container: ${REMOVE_FLAG:+yes}"
echo "  Interactive: ${INTERACTIVE_FLAG:+yes}"
echo "  Mission data: ${MISSION_DATA_MOUNT:+mounted}"
echo "  Debug mode: ${DEBUG_MOUNT:+enabled}"
echo "  Command: ${COMMAND:-default shell}"
echo ""

docker run --privileged \
    $INTERACTIVE_FLAG \
    $REMOVE_FLAG \
    --volume=$XSOCK:/root/.X11-unix:rw \
    --volume=$XAUTH:/root/.docker.xauth:rw \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=/root/.docker.xauth" \
    --env="DISPLAY=$DISPLAY" \
    --ulimit rtprio=99 \
    --cap-add=sys_nice \
    --gpus all \
    --volume $SSH_AUTH_SOCK:/ssh-agent \
    --env SSH_AUTH_SOCK=/ssh-agent \
    "${ENV_VARS[@]}" \
    $MISSION_DATA_MOUNT \
    $DEBUG_MOUNT \
    $FULL_IMAGE \
    $COMMAND

