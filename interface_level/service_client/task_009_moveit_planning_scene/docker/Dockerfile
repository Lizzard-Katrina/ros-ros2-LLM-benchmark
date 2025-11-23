FROM ros:noetic-ros-core

# Install MoveIt and dependencies
RUN apt-get update && apt-get install -y \
    ros-noetic-moveit \
    ros-noetic-moveit-ros-planning \
    ros-noetic-moveit-ros-planning-interface \
    ros-noetic-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

# Create catkin workspace
RUN mkdir -p /workspace/catkin_ws/src
WORKDIR /workspace/catkin_ws

# Copy ROS1 code
COPY ../ros1_code /workspace/catkin_ws/src/moveit_planning_scene_demo

# Build workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Copy setup script
COPY setup.sh /workspace/setup.sh
RUN chmod +x /workspace/setup.sh

ENTRYPOINT ["/workspace/setup.sh"]
