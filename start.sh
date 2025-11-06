#!/bin/bash
# Terminal Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

current_folder=$PWD
cd ..
parent_folder=$PWD
cd $current_folder

BRANCH_NAME="${1:-test/state_sharing}"
IMAGE_VERSION="${BRANCH_NAME//\//.}"
IMAGE_NAME="px4_x86_64:${IMAGE_VERSION}"
echo the version of the image to run is $IMAGE_NAME

# Allow Docker to interact with the X server
echo -e "${YELLOW}[INFO] Granting access to X server for Docker...${NC}"
xhost +local:root > /dev/null

GPU_ARGS=() 
GPU_ARGS+=(--gpus all) 
GPU_ARGS+=(-e NVIDIA_VISIBLE_DEVICES=all) 
GPU_ARGS+=(-e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute)

docker run --rm -it --gpus all --runtime=nvidia --network host --privileged \
    --privileged \
    --env="DISPLAY" \
    --workdir="/app" \
    --volume="$current_folder:/app" \
    --volume="/dev:/dev" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v /etc/hosts:/etc/hosts \
    "${GPU_ARGS[@]}" \
    $IMAGE_NAME \
    "$@"

# Revoke access to the X server after the run
echo -e "${YELLOW}[INFO] Revoking access to X server...${NC}"
xhost -local:root > /dev/null

echo -e "${GREEN}[INFO] Docker container exited.${NC}"
