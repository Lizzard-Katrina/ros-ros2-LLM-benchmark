FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    lsb-release curl git wget build-essential \
    python3 python3-pip software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# ROS1 Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update && apt-get install -y ros-noetic-desktop-full

RUN rosdep init || true
RUN rosdep update

ENV ROS1_SETUP="/opt/ros/noetic/setup.sh"

# ROS2 Foxy
RUN apt-get update && apt-get install -y ros-foxy-desktop python3-colcon-common-extensions
ENV ROS2_SETUP="/opt/ros/foxy/setup.sh"

# Workspaces
RUN mkdir -p /ros1_ws/src
RUN mkdir -p /ros2_ws/src

COPY ../ros1_code /ros1_ws/src/task_001_basic_param
COPY ../expected_ros2 /ros2_ws/src/task_001_basic_param

# Build ROS1
RUN /bin/bash -c "source ${ROS1_SETUP} && cd /ros1_ws && catkin_make"

# Build ROS2
RUN /bin/bash -c "source ${ROS2_SETUP} && cd /ros2_ws && colcon build"

CMD ["/bin/bash"]
