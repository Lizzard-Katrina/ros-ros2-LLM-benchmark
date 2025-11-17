FROM ros:noetic-ros-base

# Install ROS1 LaserScan deps (already included in sensor_msgs)
RUN apt-get update && apt-get install -y \
    python3-roslaunch \
    python3-rospy \
    python3-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 Foxy
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release
RUN curl -s https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos -o ros2.repos
RUN apt update && apt install -y ros-foxy-desktop

COPY setup.sh /setup.sh
RUN chmod +x /setup.sh
