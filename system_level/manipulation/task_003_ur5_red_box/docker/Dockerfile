FROM ros:noetic-ros-base

# Install dependencies
RUN apt-get update && apt-get install -y \
    ros-noetic-moveit \
    ros-noetic-robot-state-publisher \
    build-essential \
    cmake \
    git \
    && rm -rf /var/lib/apt/lists/*

# Copy ROS1 code
COPY ros1_code /ros_workspace/src/ros1_code

# Build ROS1 workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /ros_workspace && catkin_make"
