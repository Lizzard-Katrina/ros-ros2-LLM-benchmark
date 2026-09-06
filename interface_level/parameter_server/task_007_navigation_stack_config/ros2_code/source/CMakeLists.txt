cmake_minimum_required(VERSION 3.8)
project(task_007_navigation_stack_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)

add_executable(amcl_node amcl_node.cpp)
ament_target_dependencies(amcl_node rclcpp rcl_interfaces)

install(TARGETS amcl_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()