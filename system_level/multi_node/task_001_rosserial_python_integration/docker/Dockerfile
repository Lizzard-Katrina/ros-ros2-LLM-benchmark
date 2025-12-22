FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic

# ---------- system dependencies ----------
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-serial \
    ros-noetic-rosserial-python \
    ros-noetic-rosserial-msgs \
    ros-noetic-std-msgs \
    && rm -rf /var/lib/apt/lists/*

# ---------- workspace ----------
WORKDIR /root/ws_rosserial

# Copy benchmark code
COPY ros1_code/ src/ros1_code/

# ---------- setup ----------
COPY docker/setup.sh /root/setup.sh
RUN chmod +x /root/setup.sh

ENTRYPOINT ["/root/setup.sh"]
CMD ["bash"]
