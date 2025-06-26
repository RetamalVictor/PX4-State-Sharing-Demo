# Makefile for building ROS 2 workspace with colcon

# Use the current directory as the workspace root
WS := /app/workspace

.PHONY: all clean build rebuild

all: build

clean:
	# Remove build artifacts from the workspace root
	rm -rf $(WS)/build $(WS)/install $(WS)/log

build:
	# Build the ROS2 workspace in-place
	cd $(WS) && colcon build --symlink-install

rebuild: clean build