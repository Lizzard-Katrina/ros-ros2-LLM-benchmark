FROM ubuntu:20.04

SHELL ["/bin/bash", "-c"]

# Copy setup script
COPY setup.sh /setup.sh
RUN chmod +x /setup.sh && /setup.sh

# Create workspace folders
RUN mkdir -p /ros1_ws/src /ros2_ws/src

WORKDIR /ros1_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && colcon build"
