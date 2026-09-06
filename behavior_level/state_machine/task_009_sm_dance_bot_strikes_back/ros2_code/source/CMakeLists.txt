cmake_minimum_required(VERSION 3.8)
project(task_009_sm_dance_bot_strikes_back)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(angles REQUIRED)

set(DEPS
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  realtime_tools
  angles
)

add_library(odom_tracker SHARED
  odom_tracker.cpp
)

target_include_directories(odom_tracker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(odom_tracker ${DEPS})

add_executable(odom_tracker_node odom_tracker_node_main.cpp)
target_include_directories(odom_tracker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(odom_tracker_node odom_tracker)
ament_target_dependencies(odom_tracker_node ${DEPS})

install(TARGETS odom_tracker
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS odom_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()