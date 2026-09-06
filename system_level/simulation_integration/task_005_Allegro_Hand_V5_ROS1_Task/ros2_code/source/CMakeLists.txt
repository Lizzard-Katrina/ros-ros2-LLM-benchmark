cmake_minimum_required(VERSION 3.8)
project(task_005_Allegro_Hand_V5_ROS1_Task)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 and Gazebo dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

# Build the IMU plugin library
add_library(gazebo_imu_plugin SHARED
  gazebo_imu_plugin.cpp
)

target_include_directories(gazebo_imu_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(gazebo_imu_plugin
  rclcpp
  sensor_msgs
  gazebo_ros
  gazebo_dev
)

target_link_libraries(gazebo_imu_plugin
  ${GAZEBO_LIBRARIES}
)

# Install
install(TARGETS gazebo_imu_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()