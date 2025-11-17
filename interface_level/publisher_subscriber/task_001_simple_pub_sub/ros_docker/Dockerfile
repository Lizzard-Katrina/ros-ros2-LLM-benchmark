# === Base image ===
FROM ros:foxy

# Install build tools
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Create a workspace
WORKDIR /root/ros2_ws
RUN mkdir -p src/task_001_simple_pub_sub

# Copy your ROS2 package in
COPY expected_ros2_code/ src/task_001_simple_pub_sub/

# Source ROS2 + build
RUN . /opt/ros/foxy/setup.sh && \
    colcon build

# Source workspace at container start
CMD ["/bin/bash"]
