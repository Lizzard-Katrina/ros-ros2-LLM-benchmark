FROM ros:noetic

RUN apt-get update && apt-get install -y \
    python3-rosdep python3-catkin-tools ros-noetic-actionlib-msgs

RUN rosdep init || true && rosdep update

WORKDIR /root/ws
