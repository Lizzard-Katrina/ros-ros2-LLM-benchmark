cmake_minimum_required(VERSION 3.8)
project(task_006_swarm_ros_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(bridge_node
  src/bridge_node.cpp
)

target_include_directories(bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(bridge_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

install(TARGETS bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_runtime_ros2 ${CMAKE_CURRENT_SOURCE_DIR}/test_runtime_ros2.py
    TIMEOUT 30
  )
endif()

ament_package()