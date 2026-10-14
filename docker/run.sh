#!/bin/bash
# run.sh - Start ROS2 container (plug-and-play with native Webots on WSL2)

# Stop any old container
docker rm -f rover_dev 2>/dev/null || true

# Start container in detached mode
docker compose up -d

# Attach to container bash
docker exec -it rover_dev bash
