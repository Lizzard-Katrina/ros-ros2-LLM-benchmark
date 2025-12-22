FROM ros:noetic

RUN apt-get update && apt-get install -y \
    ros-noetic-moveit \
    ros-noetic-gazebo-ros \
    ros-noetic-vision-opencv \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /catkin_ws
COPY ros1_code src/task_002
COPY docker/setup.sh /setup.sh
RUN chmod +x /setup.sh

CMD ["/setup.sh"]
