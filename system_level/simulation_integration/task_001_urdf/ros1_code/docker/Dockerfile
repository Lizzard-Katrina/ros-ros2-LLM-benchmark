# Use ROS Noetic base image
FROM ros:noetic-ros-base

# Set environment
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Install dependencies
RUN apt-get update && apt-get install -y \
    ros-noetic-ros-base \
    ros-noetic-xacro \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-rviz \
    ros-noetic-gazebo-ros \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Copy the urdf_tutorial package into workspace
WORKDIR /root/catkin_ws/src
COPY urdf_tutorial ./urdf_tutorial

# Build workspace
WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Set entrypoint to ROS
ENTRYPOINT ["/bin/bash", "-c", "source /root/catkin_ws/devel/setup.bash && exec bash"]
