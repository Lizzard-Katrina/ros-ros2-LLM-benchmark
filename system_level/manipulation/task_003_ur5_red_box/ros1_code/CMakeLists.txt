cmake_minimum_required(VERSION 2.8.3)
project(robotic_arm_algorithms)

## Compile as C++11, supported in ROS Kinetic and newer
if("$ENV{ROS_DISTRO}" STRLESS "lunar")
  add_compile_options( -std=c++11 ${GAZEBO_CXX_FLAGS} )
endif()

# TODO
# 1. Finding ROS 2 dependencies (rclpy, rosidl_default_generators, etc.).
# 2. Generating interfaces for 'srv/SetJointStates.srv'.
# 3. Installing Python scripts to 'lib/${PROJECT_NAME}'.
# 4. Exporting the package using 'ament_package()'.
#END OF TODO
