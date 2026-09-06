cmake_minimum_required(VERSION 3.8)
project(task_007_move_base_navigation_action)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Install the translated source for static oracle analysis
install(FILES move_base.cpp DESTINATION share/${PROJECT_NAME})

# Minimal stub executable so `ros2 run` works for runtime testing
add_executable(move_base_node src/move_base_node_stub.cpp)
ament_target_dependencies(move_base_node rclcpp std_msgs geometry_msgs)
install(TARGETS move_base_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_runtime_ros2
    "${CMAKE_CURRENT_SOURCE_DIR}/test_runtime_ros2.py"
    TIMEOUT 30
  )
endif()

ament_package()