FROM ros:noetic

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    build-essential \
    ros-noetic-image-common

# Create workspace
RUN mkdir -p /root/ros1_ws/src

COPY ros1_code /root/ros1_ws/src/task_008_camera_object_detection_yolo

RUN source /opt/ros/noetic/setup.bash && \
    cd /root/ros1_ws && \
    catkin_make

COPY setup.sh /setup.sh
RUN chmod +x /setup.sh

ENTRYPOINT ["/setup.sh"]
