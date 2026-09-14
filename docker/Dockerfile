FROM ros:jazzy-ros-base

# Avoid interactive prompts during apt installs
ENV DEBIAN_FRONTEND=noninteractive

# System + ROS2 build tooling
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-jazzy-ament-cmake \
    git \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep (skip if already done in base image)
RUN rosdep update

# Your workspace will be mounted here at runtime
WORKDIR /ros2_ws

# Source ROS2 automatically in every new shell
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

# Keep container alive / drop into bash by default
CMD ["bash"]