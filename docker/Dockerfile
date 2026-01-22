# Base image: ROS2 Jazzy Desktop
FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Install basic development tools and dependencies
RUN apt update && apt install -y \
    git \
    nano \
    curl \
    wget \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    lsb-release \
    gnupg2 \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# Install Webots directly from official .deb
RUN wget https://github.com/cyberbotics/webots/releases/download/R2025b/webots_2025b_amd64.deb -O /tmp/webots.deb \
    && dpkg -i /tmp/webots.deb || apt-get install -f -y \
    && rm /tmp/webots.deb

# Optional: Install ROS2 Webots bridge (if available for Jazzy)
# If not in repo, build from source in /workspace/src later
RUN apt update && apt install -y ros-jazzy-webots-ros2 || true

# Set workspace
WORKDIR /workspace

# Auto-source ROS2 on container start
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
