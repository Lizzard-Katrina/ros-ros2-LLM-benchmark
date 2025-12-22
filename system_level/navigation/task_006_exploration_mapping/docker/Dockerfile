FROM ros:noetic-ros-base

# Install necessary packages for navigation and SLAM
RUN apt-get update && apt-get install -y \
    ros-noetic-navigation \
    ros-noetic-slam-gmapping \
    ros-noetic-map-server \
    && rm -rf /var/lib/apt/lists/*

COPY ros1_code /catkin_ws/src/exploration_mapping_task
COPY docker/setup.sh /setup.sh
RUN chmod +x /setup.sh
CMD ["/setup.sh"]
