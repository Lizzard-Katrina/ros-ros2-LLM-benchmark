FROM ros:noetic-ros-base

SHELL ["/bin/bash", "-c"]

# ---------- System dependencies ----------
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-catkin-tools \
    ros-noetic-moveit \
    ros-noetic-tf \
    ros-noetic-tf2-ros \
    ros-noetic-geometry-msgs \
    ros-noetic-moveit-msgs \
    ros-noetic-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

# ---------- Workspace ----------
ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS/src

# ---------- Copy benchmark ROS1 code ----------
COPY ros1_code/src ./ez_pick_and_place_benchmark

# ---------- Build ----------
WORKDIR $CATKIN_WS
RUN source /opt/ros/noetic/setup.bash && \
    catkin_make

# ---------- Runtime ----------
COPY docker/setup.sh /setup.sh
RUN chmod +x /setup.sh

ENTRYPOINT ["/setup.sh"]
CMD ["bash"]
