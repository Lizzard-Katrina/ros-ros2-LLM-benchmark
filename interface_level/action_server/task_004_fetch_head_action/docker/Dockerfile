FROM ros:noetic-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y ros-noetic-fetch-head-msgs

RUN mkdir -p /root/ros1_ws/src
WORKDIR /root/ros1_ws/src

COPY ../ros1_code ./head_action_server

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd /root/ros1_ws && catkin_make"
