# Base ROS2 Jazzy Desktop
FROM osrf/ros:jazzy-desktop

# Install basic dev tools
RUN apt update && apt install -y \
    git \
    nano \
    curl \
    wget \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# Install dependencies for Webots GUI
RUN apt update && apt install -y \
    libxcb1 libxcb-xinerama0 libxcb-cursor0 libx11-xcb1 \
    libxrender1 libxext6 libxi6 libxrandr2 libxfixes3 libxcursor1 \
    libxcomposite1 libxdamage1 libxss1 libglu1-mesa

# Install Webots from Cyberbotics official repo
RUN mkdir -p /etc/apt/keyrings && \
    wget -qO /etc/apt/keyrings/Cyberbotics.asc https://cyberbotics.com/Cyberbotics.asc && \
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/" \
        > /etc/apt/sources.list.d/Cyberbotics.list && \
    apt update && \
    apt install -y webots && \
    rm -rf /var/lib/apt/lists/*

# Set workspace
WORKDIR /workspace
