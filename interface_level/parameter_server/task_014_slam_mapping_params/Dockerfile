FROM ros:noetic-ros-base

RUN apt-get update && apt-get install -y \
    python3-rosdep \
    python3-yaml \
    ros-noetic-slam-gmapping

WORKDIR /root/ws
COPY ../ros1_code ./ros1_code

CMD ["bash"]
