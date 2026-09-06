cmake_minimum_required(VERSION 3.8)
project(task_008_clear_costmaps)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

# Main executable (the gtest-based clear tester)
add_executable(clear_tester clear_tester.cpp)
ament_target_dependencies(clear_tester
  rclcpp
  std_srvs
  tf2_ros
  tf2
)

# Link gtest
find_package(ament_cmake_gtest REQUIRED)
target_link_libraries(clear_tester gtest gtest_main pthread)

install(TARGETS clear_tester
  DESTINATION lib/${PROJECT_NAME}
)

# Python tests
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_runtime_ros2
    "${CMAKE_CURRENT_SOURCE_DIR}/test_runtime_ros2.py"
    TIMEOUT 30
  )
endif()

ament_package()