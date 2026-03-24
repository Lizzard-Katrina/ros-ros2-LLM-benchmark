cmake_minimum_required(VERSION 3.5)
project(task_006)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)

add_executable(stereo_sync stereo_sync.cpp)
ament_target_dependencies(stereo_sync rclcpp sensor_msgs message_filters)

install(TARGETS
  stereo_sync
  DESTINATION lib/${PROJECT_NAME})

ament_package()
