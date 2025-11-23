FROM ros:noetic-ros-core

# Install Gazebo and ROS Gazebo packages
RUN apt-get update && apt-get install -y \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    && rm -rf /var/lib/apt/lists/*

# Create catkin workspace
RUN mkdir -p /workspace/catkin_ws/src
WORKDIR /workspace/catkin_ws

# Copy ROS1 code
COPY ../ros1_code /workspace/catkin_ws/src/gazebo_state_demo

# Build
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Copy setup script
COPY setup.sh /workspace/setup.sh
RUN chmod +x /workspace/setup.sh

ENTRYPOINT ["/workspace/setup.sh"]
