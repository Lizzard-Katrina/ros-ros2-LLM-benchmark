FROM navigation_base:latest
RUN apt-get update && apt-get install -y \
    ros-noetic-turtlebot3-gazebo \
    ros-noetic-turtlebot3-navigation \
    ros-noetic-turtlebot3-bringup \
    && rm -rf /var/lib/apt/lists/*

COPY ros1_code /catkin_ws/src/benchmark_turtlebot3_nav
COPY docker/setup.sh /setup.sh
RUN chmod +x /setup.sh
CMD ["/setup.sh"]
