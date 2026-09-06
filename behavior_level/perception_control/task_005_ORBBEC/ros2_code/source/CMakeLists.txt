cmake_minimum_required(VERSION 3.8)
project(task_005_ORBBEC)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# The actual node depends on OrbbecSDK which is not available in standard ROS2.
# We install the source file for static analysis / oracle tests.
install(
  FILES ob_camera_node.cpp
  DESTINATION share/${PROJECT_NAME}
)

ament_package()