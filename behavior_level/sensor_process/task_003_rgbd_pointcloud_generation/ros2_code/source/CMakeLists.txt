cmake_minimum_required(VERSION 3.8)
project(task_003_rgbd_pointcloud_generation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_geometry REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(depth_image_proc REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)

install(FILES point_cloud_xyzrgb.cpp
  DESTINATION share/${PROJECT_NAME}/)

ament_package()