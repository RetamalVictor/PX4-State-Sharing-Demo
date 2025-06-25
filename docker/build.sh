#!/usr/bin/env bash

# ANSI color codes
GREEN='\033[0;32m'   # green text :contentReference[oaicite:0]{index=0}
RED='\033[0;31m'     # red text :contentReference[oaicite:1]{index=1}
YELLOW='\033[1;33m'  # yellow text :contentReference[oaicite:2]{index=2}
NC='\033[0m'         # no color / reset :contentReference[oaicite:3]{index=3}

BRANCH_NAME="${1:-test/px4_pr}"
IMAGE_VERSION=px4_custom
IMAGE_NAME=px4_x86_64:$IMAGE_VERSION

# Announce build start
echo -e "${YELLOW}==> Starting Docker build for image ${GREEN}${IMAGE_NAME}${YELLOW} <==${NC}"
export DOCKER_BUILDKIT=1
# Run the build
if docker build --no-cache \
       --ssh default \
       --build-arg BRANCH_NAME="${BRANCH_NAME}" \
       --network host \
       --rm=false \
       -t "${IMAGE_NAME}" \
       "$(dirname "$0")"; then
    # Success
    echo -e "${GREEN}✓ Successfully built image ${IMAGE_NAME}${NC}"
else
    # Failure
    echo -e "${RED}✗ Docker build failed for image ${IMAGE_NAME}${NC}" >&2
    exit 1
fi