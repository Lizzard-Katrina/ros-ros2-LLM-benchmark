FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

# -----------------------------
# Install basics
# -----------------------------
RUN apt update && apt install -y \
    wget curl git lsb-release build-essential \
    python3-colcon-common-extensions \
    python3-rosdep python3-pip

# -----------------------------
# ROS1 (Noetic)
# -----------------------------
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN apt update && apt install -y ros-noetic-desktop-full

# -----------------------------
# ROS2 (Foxy)
# -----------------------------
RUN sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2.list'
RUN apt update && apt install -y ros-foxy-desktop ros-foxy-ros-base

# -----------------------------
# Source setup
# -----------------------------
SHELL ["/bin/bash", "-c"]

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc

# -----------------------------
# Prepare workspaces
# -----------------------------
RUN mkdir -p /root/ros1_ws/src
RUN mkdir -p /root/ros2_ws/src
RUN mkdir -p /root/bridge_ws/src

# Copy benchmark code skeletons (mount during run)
WORKDIR /root

# -----------------------------
# rosdep init
# -----------------------------
RUN rosdep init || true
RUN rosdep update

# -----------------------------
# Setup script
# -----------------------------
COPY setup.sh /root/setup.sh
RUN chmod +x /root/setup.sh
