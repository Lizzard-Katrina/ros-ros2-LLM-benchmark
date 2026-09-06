cmake_minimum_required(VERSION 3.8)
project(task_007_hector_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Header-only / source library for the migrated callback
add_library(hector_mapping_ros_lib SHARED
  src/HectorMappingRos.cpp
)

target_include_directories(hector_mapping_ros_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(hector_mapping_ros_lib
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  builtin_interfaces
)

# Install the library
install(TARGETS hector_mapping_ros_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# Simple node executable
add_executable(hector_mapping_node src/main.cpp)
target_link_libraries(hector_mapping_node hector_mapping_ros_lib)
ament_target_dependencies(hector_mapping_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  builtin_interfaces
)

install(TARGETS hector_mapping_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()