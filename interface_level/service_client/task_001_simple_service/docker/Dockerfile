FROM ubuntu:20.04
ENV DEBIAN_FRONTEND=noninteractive

# 基础工具
RUN apt-get update && apt-get install -y \
    sudo curl wget git lsb-release gnupg2 python3-pip build-essential \
    python3-colcon-common-extensions python3-rosdep python3-rosinstall-generator python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# ROS1 Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update && apt-get install -y ros-noetic-desktop-full
RUN rosdep init && rosdep update

# ROS2 Humble
RUN apt update && apt install -y software-properties-common
RUN add-apt-repository universe
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add -
RUN sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'
RUN apt update && apt install -y ros-humble-desktop

# 创建工作区
RUN mkdir -p /workspace/ros1_ws/src /workspace/ros2_ws/src
WORKDIR /workspace

# 初始化 ROS1 工作区
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd ros1_ws && catkin_make"

# 初始化 ROS2 工作区
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd ros2_ws && colcon build"

ENV ROS1_WS=/workspace/ros1_ws
ENV ROS2_WS=/workspace/ros2_ws
CMD ["/bin/bash"]
