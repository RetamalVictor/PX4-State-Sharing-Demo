#!/usr/bin/env bash

# ANSI color codes
GREEN='\033[0;32m'   # green text
RED='\033[0;31m'     # red text
YELLOW='\033[1;33m'  # yellow text
NC='\033[0m'         # no color / reset

BRANCH_NAME="${1:-test/px4_pr}"
IMAGE_VERSION="test.px4_pr_v2"
IMAGE_NAME="px4_x86_64:${IMAGE_VERSION}"

# Announce build start
echo -e "${YELLOW}==> Starting Docker build for image ${GREEN}${IMAGE_NAME}${YELLOW} <==${NC}"

export DOCKER_BUILDKIT=1

# Run the build
if DOCKER_BUILDKIT=1 docker build --no-cache "$(dirname "$0")" \
     -t "${IMAGE_NAME}" \
     --network host \
     --rm=false \
     --ssh default \
     --build-arg BRANCH_NAME="${BRANCH_NAME}"
then
    # Success
    echo -e "${GREEN}✓ Successfully built image ${IMAGE_NAME}${NC}"
else
    # Failure
    echo -e "${RED}✗ Docker build failed for image ${IMAGE_NAME}${NC}" >&2
    exit 1
fi
