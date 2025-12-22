FROM ros:noetic

RUN apt-get update && apt-get install -y \
    ros-noetic-moveit \
    ros-noetic-moveit-task-constructor \
    ros-noetic-franka-description \
    ros-noetic-franka-gazebo \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /catkin_ws
