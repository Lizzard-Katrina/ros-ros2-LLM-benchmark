# Task 005 Dockerfile
FROM osrf/ros:noetic-desktop-full

# Set environment
ENV ROS_WS=/root/catkin_ws

# Create workspace
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS

# Install dependencies
RUN apt-get update && \
    apt-get install -y python3-pip python3-catkin-tools && \
    pip3 install rospkg catkin_pkg

# Clone Interbotix ROS manipulators
RUN git clone -b noetic-devel https://github.com/Interbotix/interbotix_ros_manipulators.git src/interbotix_ros_manipulators

# Build workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Source ROS
RUN echo "source $ROS_WS/devel/setup.bash" >> /root/.bashrc
