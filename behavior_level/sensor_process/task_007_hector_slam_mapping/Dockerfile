FROM ros:noetic

RUN apt-get update && apt-get install -y \
    ros-noetic-hector-slam \
    python3-roslaunch \
    python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws

COPY ros1_code/ src/ros1_task_007/

RUN . /opt/ros/noetic/setup.sh && catkin_make

COPY setup.sh /root/

ENTRYPOINT ["bash", "/root/setup.sh"]
