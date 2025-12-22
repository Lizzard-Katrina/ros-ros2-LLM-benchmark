FROM ros:noetic

# Create workspace
RUN mkdir -p /root/ros1_ws/src
WORKDIR /root/ros1_ws/src

# Copy package
COPY ../ros1_code ./laser_obstacle_detection

# Build ROS1 workspace
WORKDIR /root/ros1_ws
RUN . /opt/ros/noetic/setup.sh && catkin_make

# Source environment at container start
CMD ["/bin/bash"]
