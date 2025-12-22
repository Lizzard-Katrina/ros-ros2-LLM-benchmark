FROM ros:noetic

SHELL ["/bin/bash", "-c"]

RUN apt update && apt install -y \
    ros-noetic-image-common \
    ros-noetic-image-pipeline \
    python3-colcon-common-extensions \
    build-essential

RUN mkdir -p /root/ros1_ws/src

COPY ros1_code /root/ros1_ws/src/stereo_task

RUN source /opt/ros/noetic/setup.bash && \
    cd /root/ros1_ws && \
    catkin_make

COPY setup.sh /setup.sh
RUN chmod +x /setup.sh

ENTRYPOINT ["/setup.sh"]
