FROM ros:noetic-ros-base

# Install navigation packages
RUN apt-get update && apt-get install -y \
    ros-noetic-navigation \
    ros-noetic-turtlebot3-navigation \
    && rm -rf /var/lib/apt/lists/*

COPY ros1_code /catkin_ws/src/plugin_replacement_task
COPY docker/setup.sh /setup.sh
RUN chmod +x /setup.sh
CMD ["/setup.sh"]
