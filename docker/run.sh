#!/bin/bash
# Detect if Windows 10 (VcXsrv) or Windows 11 (WSLg) or Linux
OS_NAME=$(uname -r)

if [[ "$OS_NAME" == *"microsoft"* ]]; then
    # Inside WSL
    WIN_VER=$(cmd.exe /c ver | tr -d '\r')
    if [[ "$WIN_VER" == *"10."* ]]; then
        echo "Windows 10 detected: using VcXsrv for GUI"
        export HOST_DISPLAY=host.docker.internal:0.0
    else
        echo "Windows 11 detected: using WSLg"
        export HOST_DISPLAY=$DISPLAY
    fi
else
    # Linux / Mac
    export HOST_DISPLAY=$DISPLAY
fi

# Start container and open bash
docker compose up -d
docker exec -it rover_dev bash
