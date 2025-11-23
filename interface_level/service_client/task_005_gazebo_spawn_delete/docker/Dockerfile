FROM ros:noetic-ros-core

# 安装 Gazebo 和 ROS Gazebo 插件
RUN apt-get update && apt-get install -y \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    && rm -rf /var/lib/apt/lists/*

# 创建 catkin workspace
RUN mkdir -p /workspace/catkin_ws/src
WORKDIR /workspace/catkin_ws

# 拷贝 ros1_code
COPY ../ros1_code /workspace/catkin_ws/src/gazebo_service_demo

# 编译
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# setup 脚本
COPY setup.sh /workspace/setup.sh
RUN chmod +x /workspace/setup.sh
ENTRYPOINT ["/workspace/setup.sh"]
