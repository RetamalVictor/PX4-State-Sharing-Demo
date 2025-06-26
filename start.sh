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

# IMAGE_NAME=px4_x86_64:px4_custom_v2
IMAGE_NAME=px4_x86_64:test.px4_pr_v2
# IMAGE_NAME=repo.arrc.tii.ae/gnc-docker/px4_x86_64:test.px4_pr
echo the version of the image to run is $IMAGE_NAME

# Allow Docker to interact with the X server
echo -e "${YELLOW}[INFO] Granting access to X server for Docker...${NC}"
xhost +local:root > /dev/null

docker run -it --rm \
    --privileged \
    --env="DISPLAY" \
    --workdir="/app" \
    --volume="$current_folder:/app" \
    --volume="/dev:/dev" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --network host \
    $IMAGE_NAME \
    "$@"

# Revoke access to the X server after the run
echo -e "${YELLOW}[INFO] Revoking access to X server...${NC}"
xhost -local:root > /dev/null

echo -e "${GREEN}[INFO] Docker container exited.${NC}"
