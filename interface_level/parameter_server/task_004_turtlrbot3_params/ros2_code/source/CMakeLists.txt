cmake_minimum_required(VERSION 3.8)
project(task_004_turtlrbot3_params)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

# NOTE: the verbatim turtlebot3.cpp cannot be compiled here -- it depends on the
# turtlebot3_node package headers, DynamixelSDK and OpenCR hardware. It is shipped
# as a source file for the static oracle (tests/test_oracle_ros2.py).
#
# profile_accel_param_node carries the exact migrated parameter-event conversion
# logic from turtlebot3.cpp so that pattern is actually built and executed by
# tests/test_runtime_ros2.py.
add_executable(profile_accel_param_node src/profile_accel_param_node.cpp)
ament_target_dependencies(profile_accel_param_node rclcpp rcl_interfaces std_msgs)

install(TARGETS profile_accel_param_node
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES turtlebot3.cpp DESTINATION share/${PROJECT_NAME})

ament_package()
