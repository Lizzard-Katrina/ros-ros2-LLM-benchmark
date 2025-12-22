FROM ros:noetic-ros-base

RUN apt-get update && apt-get install -y \
    ros-noetic-navigation \
    ros-noetic-slam-gmapping \
    ros-noetic-map-server \
    ros-noetic-move-base \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /catkin_ws

COPY ros1_code src/task_008_unknown_env_exploration

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

CMD ["bash"]
