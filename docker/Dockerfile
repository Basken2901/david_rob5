FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-jazzy-ur \
    ros-jazzy-ur-robot-driver \
    ros-jazzy-moveit \
    ros-jazzy-rviz2 \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-rmw-cyclonedds-cpp \
    git \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# Note: the base ros image already runs rosdep init; only update is needed
RUN rosdep update

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WORKDIR /ros2_ws
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
CMD ["bash"]