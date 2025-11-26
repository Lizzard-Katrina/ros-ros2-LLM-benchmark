FROM ros:noetic-ros-core

# 安装必要工具
RUN apt-get update && \
    apt-get install -y python3-pip python3-catkin-tools ros-noetic-actionlib \
                       ros-noetic-move-base ros-noetic-turtlebot3-msgs \
                       && rm -rf /var/lib/apt/lists/*

# 设置工作目录
WORKDIR /ros_ws

# 拷贝 ros1_code 并 build
COPY ../ros1_code ./src
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# 设置 entrypoint
CMD ["/bin/bash"]
