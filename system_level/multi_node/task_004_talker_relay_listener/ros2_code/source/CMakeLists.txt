cmake_minimum_required(VERSION 3.8)
project(task_004_talker_relay_listener)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

# Server
add_executable(add_two_ints_server src/add_two_inits_server.cpp)
ament_target_dependencies(add_two_ints_server rclcpp example_interfaces)

# Client
add_executable(add_two_ints_client src/add_two_inits_client.cpp)
ament_target_dependencies(add_two_ints_client rclcpp example_interfaces)

# Babbler
add_executable(babbler src/babbler.cpp)
ament_target_dependencies(babbler rclcpp std_msgs)

install(TARGETS
  add_two_ints_server
  add_two_ints_client
  babbler
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_runtime_ros2
    ${CMAKE_CURRENT_SOURCE_DIR}/test_runtime_ros2.py
    TIMEOUT 60
  )
endif()

ament_package()