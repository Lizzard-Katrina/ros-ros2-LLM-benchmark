cmake_minimum_required(VERSION 3.8)
project(task_002_dynamic_robot_configuration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(turtlesim_node src/turtlesim.cpp)
ament_target_dependencies(turtlesim_node rclcpp)

install(TARGETS
  turtlesim_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_runtime_ros2 test_runtime_ros2.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    TIMEOUT 30
  )
  ament_add_pytest_test(test_oracle test/test_oracle.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    TIMEOUT 30
  )
endif()

ament_package()