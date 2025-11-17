set -e

# Source ROS1 and ROS2
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash

# Build ROS1 workspace
cd /root/ros1_ws
rosdep install --from-paths src -r -y
catkin_make

# Build ROS2 workspace
cd /root/ros2_ws
rosdep install --from-paths src -r -y
colcon build

# Build bridge
cd /root/bridge_ws
git clone https://github.com/ros2/ros1_bridge src/ros1_bridge
rosdep install --from-paths src -r -y
colcon build --packages-select ros1_bridge --cmake-force-configure
