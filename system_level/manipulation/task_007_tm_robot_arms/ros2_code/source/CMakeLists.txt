cmake_minimum_required(VERSION 3.8)
project(task_007_tm_robot_arms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Install the C++ source files so the oracle test can find them
install(FILES
  tm_communication.cpp
  tm_ros_service.cpp
  DESTINATION share/${PROJECT_NAME}
)

# Install the Python demo script
install(PROGRAMS
  ask_item_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()