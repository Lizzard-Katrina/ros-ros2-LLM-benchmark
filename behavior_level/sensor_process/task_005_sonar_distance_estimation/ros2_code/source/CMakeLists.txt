cmake_minimum_required(VERSION 3.8)
project(task_005_sonar_distance_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Install the turtle.cpp source file so tests can find it
install(FILES turtle.cpp DESTINATION share/${PROJECT_NAME}/)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/kinematics_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()