FROM ros:noetic

SHELL ["/bin/bash", "-c"]

# ROS2 tools (optional placeholder)
RUN apt update && apt install -y \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    python3-colcon-common-extensions

# Create ROS1 workspace
RUN mkdir -p /root/ros1_ws/src

# Copy benchmark task
COPY ros1_code /root/ros1_ws/src/imu_task

# Build ROS1 workspace
RUN source /opt/ros/noetic/setup.bash && \
    cd /root/ros1_ws && \
    catkin_make

# Setup helper script
COPY setup.sh /setup.sh
RUN chmod +x /setup.sh

ENTRYPOINT ["/setup.sh"]
