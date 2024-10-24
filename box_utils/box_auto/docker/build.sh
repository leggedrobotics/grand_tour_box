#!/bin/bash

# Default values
DOCKERFILE_PREFIX="box"
TAG_PREFIX="rslethz/grand_tour_box"
SSH_KEY="$HOME/.ssh/jonfrey_ws"
REPO_PATH="$HOME/git/grand_tour_box"
DOCKER_PATH="$REPO_PATH/box_utils/box_auto/docker"

# Help function
print_usage() {
    echo "Usage: $0 [OPTIONS] <custom_tag_suffix>"
    echo "Options:"
    echo "  --kleinkram         Build kleinkram configuration"
    echo "  --kleinkram_minimal Build kleinkram minimal configuration"
    echo "  --ros2             Build ROS2 configuration"
    echo "  --full             Build full configuration"
    echo "  --help             Show this help message"
    echo ""
    echo "Example: $0 --kleinkram v1.0"
}

# Check if we have at least one argument
if [ $# -eq 0 ]; then
    print_usage
    exit 1
fi

# Parse arguments
BUILD_TYPE=""
TAG_SUFFIX=""
NO_CACHE_FLAG=""
while [[ $# -gt 0 ]]; do
    case $1 in
        --kleinkram)
            BUILD_TYPE="kleinkram"
            TAG_SUFFIX=""
            shift
            ;;
        --kleinkram_minimal)
            BUILD_TYPE="kleinkram_minimal"
            TAG_SUFFIX=""
            shift
            ;;
        --ros2)
            BUILD_TYPE="ros2"
            TAG_SUFFIX=""
            shift
            ;;
        --full)
            BUILD_TYPE="full"
            TAG_SUFFIX=""
            shift
            ;;
        --no-cache)
            NO_CACHE_FLAG="--no-cache"
            shift
            ;;
        --help)
            print_usage
            exit 0
            ;;
        *)
            echo "Error: Invalid option '$1'"
            print_usage
            exit 1
            ;;
    esac
done


# Construct dockerfile name and tag
DOCKERFILE="box_${BUILD_TYPE}.Dockerfile"
FULL_TAG="${TAG_PREFIX}:${BUILD_TYPE}${TAG_SUFFIX}"

# Check if Dockerfile exists
if [ ! -f "$DOCKER_PATH/$DOCKERFILE" ]; then
    echo "Error: Dockerfile not found at $DOCKER_PATH/$DOCKERFILE"
    exit 1
fi

# Enable BuildKit
export DOCKER_BUILDKIT=1

# Execute docker build
echo "Building with:"
echo "  Dockerfile: $DOCKERFILE"
echo "  Tag: $FULL_TAG"
echo "  Context: $DOCKER_PATH"
echo ""

docker build \
    --progress=plain \
    --ssh default="$SSH_KEY" \
    $NO_CACHE_FLAG\
    -t "$FULL_TAG" \
    -f "$DOCKER_PATH/$DOCKERFILE" \
    "$DOCKER_PATH"

# Check build status
if [ $? -eq 0 ]; then
    echo "Build completed successfully!"
    echo "Image tagged as: $FULL_TAG"
else
    echo "Build failed!"
    exit 1
fi