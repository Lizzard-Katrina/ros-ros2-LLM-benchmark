cmake_minimum_required(VERSION 3.8)
project(task_006_amcl_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/UpdatePose.action"
  DEPENDENCIES geometry_msgs
)

add_executable(amcl_node amcl_node.cpp)

ament_target_dependencies(amcl_node
  rclcpp
  rclcpp_action
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(amcl_node "${cpp_typesupport_target}")

install(TARGETS
  amcl_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_runtime_ros2 test_runtime_ros2.py)
endif()

ament_package()