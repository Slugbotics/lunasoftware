#!/bin/bash
# run.sh - Start ROS2 + Webots container (GPU + WSLg / Linux)

# Stop any old container
docker rm -f rover_dev 2>/dev/null || true

# Detect OS and set DISPLAY
OS_NAME=$(uname -r)
if [[ "$OS_NAME" == *"microsoft"* ]]; then
    # Inside WSL
    HOST_DISPLAY=$DISPLAY
else
    # Linux / Mac
    HOST_DISPLAY=$DISPLAY
fi

# Start container with GPU support
docker run -it \
  -e DISPLAY=$DISPLAY \
  -v /mnt/wslg/.X11-unix:/tmp/.X11-unix \
  -v $(pwd)/workspace:/workspace \
  --name rover_dev \
  docker-rover \
  bash
