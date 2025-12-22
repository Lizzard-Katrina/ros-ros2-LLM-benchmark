FROM ros:noetic

SHELL ["/bin/bash", "-c"]

# Optional ROS2 tools
RUN apt update && apt install -y \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    python3-colcon-common-extensions

# ROS1 workspace
RUN mkdir -p /root/ros1_ws/src

COPY ros1_code /root/ros1_ws/src/sonar_task

RUN source /opt/ros/noetic/setup.bash && \
    cd /root/ros1_ws && \
    catkin_make

COPY setup.sh /setup.sh
RUN chmod +x /setup.sh

ENTRYPOINT ["/setup.sh"]
