cmake_minimum_required(VERSION 3.8)
project(task_004_imu_odometry_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Install the ukf.cpp source file for inspection/testing
install(FILES ukf.cpp DESTINATION share/${PROJECT_NAME}/)

# Build a standalone test node that does NOT link robot_localization
# but implements the projectSigmaPoint logic directly for verification
add_executable(ukf_test_node src/ukf_test_node.cpp)
ament_target_dependencies(ukf_test_node rclcpp std_msgs)
target_include_directories(ukf_test_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
)
install(TARGETS ukf_test_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_runtime_ros2
    ${CMAKE_CURRENT_SOURCE_DIR}/test_runtime_ros2.py
    TIMEOUT 30
  )
endif()

ament_package()