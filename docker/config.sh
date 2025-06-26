# config.sh

# Default branch if not provided
BRANCH_NAME=${1:-test/px4_pr}

# Convert the branch name to image version by replacing "/" with "."
IMAGE_VERSION=$(echo $BRANCH_NAME | sed 's/\//./g')

# Define the image name based on the version
IMAGE_NAME=px4_x86_64:$IMAGE_VERSION