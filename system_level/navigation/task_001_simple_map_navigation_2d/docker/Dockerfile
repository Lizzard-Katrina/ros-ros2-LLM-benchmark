FROM ros:noetic

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    ros-noetic-navigation \
    ros-noetic-map-server \
    ros-noetic-amcl \
    ros-noetic-move-base \
    python3-tf \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /catkin_ws
RUN mkdir -p src
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

COPY ros1_code /catkin_ws/src/benchmark_nav
COPY docker/setup.sh /setup.sh

RUN chmod +x /setup.sh

CMD ["/setup.sh"]
