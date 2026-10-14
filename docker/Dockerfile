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

# Install dependencies for GUI (for Webots to display)
RUN apt update && apt install -y \
    libxcb1 libxcb-xinerama0 libxcb-cursor0 libx11-xcb1 \
    libxrender1 libxext6 libxi6 libxrandr2 libxfixes3 libxcursor1 \
    libxcomposite1 libxdamage1 libxss1 libglu1-mesa \
    && rm -rf /var/lib/apt/lists/*

# Set workspace
WORKDIR /workspace
