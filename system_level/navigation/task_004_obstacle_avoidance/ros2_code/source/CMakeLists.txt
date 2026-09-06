cmake_minimum_required(VERSION 3.8)
project(task_004_obstacle_avoidance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Include directories for custom headers
include_directories(include)

# draw_square executable
add_executable(draw_square src/draw_square.cpp)
ament_target_dependencies(draw_square
  rclcpp
  geometry_msgs
  std_srvs
  turtlesim
)

install(TARGETS
  draw_square
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(oracle_test test/test_oracle.py)
  ament_add_pytest_test(runtime_test test_runtime_ros2.py)
endif()

ament_package()