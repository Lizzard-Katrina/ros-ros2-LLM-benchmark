# 使用官方 ROS2 Humble 基础镜像（自带 Python 3.10）
FROM ros:humble

# 安装 Python/colcon/OpenCV/numpy
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-opencv \
    python3-numpy

# 安装 OpenRouter 支持的 OpenAI SDK
RUN pip3 install --upgrade pip && \
    pip3 install openai==1.51.0

# 创建 ROS2 工作空间目录（可选）
RUN mkdir -p /root/ros2_ws/src

# 默认工作目录
WORKDIR /ros_benchmark

# 启动 bash
CMD ["/bin/bash"]
