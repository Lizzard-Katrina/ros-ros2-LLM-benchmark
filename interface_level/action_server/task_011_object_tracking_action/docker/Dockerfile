FROM ros:kinetic-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y ros-kinetic-moveit ros-kinetic-pcl-ros ros-kinetic-depth-image-proc ros-kinetic-image-transport

RUN mkdir -p /root/ros1_ws/src
WORKDIR /root/ros1_ws/src

COPY ../ros1_code ./object_tracking_action_server

RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash && \
    cd /root/ros1_ws && catkin_make"
