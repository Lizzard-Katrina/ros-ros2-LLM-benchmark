cmake_minimum_required(VERSION 3.8)
project(task_008a_robot_statemachine_scaffold)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OperationMode.msg"
  "srv/SetOperationMode.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# Library
add_library(robot_control_mux_lib src/RobotControlMux.cpp)
target_include_directories(robot_control_mux_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(robot_control_mux_lib
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(robot_control_mux_lib "${cpp_typesupport_target}")

# Executable
add_executable(robot_control_mux_node src/RobotControlMuxNode.cpp)
target_include_directories(robot_control_mux_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(robot_control_mux_node robot_control_mux_lib)
ament_target_dependencies(robot_control_mux_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

install(TARGETS robot_control_mux_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS robot_control_mux_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()