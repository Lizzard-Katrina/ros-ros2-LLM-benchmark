FROM ros:noetic-ros-core

# Install build tools and necessary packages
RUN apt-get update && apt-get install -y \
    python3-catkin-tools \
    build-essential \
    ros-noetic-std-msgs \
    ros-noetic-rospy \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace
RUN mkdir -p /workspace/src
WORKDIR /workspace

# Copy ROS1 code into container
COPY ros1_code ./src/task_code

# Build workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Default command (can be overridden in tests)
CMD ["bash"]
