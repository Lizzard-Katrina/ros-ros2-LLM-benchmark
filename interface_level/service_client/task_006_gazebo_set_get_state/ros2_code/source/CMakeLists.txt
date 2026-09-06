cmake_minimum_required(VERSION 3.8)
project(task_006_gazebo_set_get_state)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(gazebo_model_states gazebo_model_states.cpp)
ament_target_dependencies(gazebo_model_states
  rclcpp
  gazebo_msgs
  geometry_msgs
)

install(TARGETS
  gazebo_model_states
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()