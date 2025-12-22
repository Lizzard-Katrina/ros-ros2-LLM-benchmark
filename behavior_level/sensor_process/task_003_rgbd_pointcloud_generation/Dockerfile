FROM ros:noetic

SHELL ["/bin/bash", "-c"]

# Install ROS2 dependencies
RUN apt update && apt install -y \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    python3-colcon-common-extensions

# Install ROS2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos -o /tmp/ros2.repos \
    && mkdir -p /opt/ros2_humble_ws/src \
    && vcs import /opt/ros2_humble_ws/src < /tmp/ros2.repos \
    || echo "Skipping ROS2 repo import"

# ROS1 workspace
RUN mkdir -p /root/ros1_ws/src

# Copy benchmark ROS1 code
COPY ros1_code /root/ros1_ws/src/rgbd_task

# Build ROS1
RUN source /opt/ros/noetic/setup.bash && \
    cd /root/ros1_ws && \
    catkin_make

# Setup script accessible inside container
COPY setup.sh /setup.sh
RUN chmod +x /setup.sh

ENTRYPOINT ["/setup.sh"]
