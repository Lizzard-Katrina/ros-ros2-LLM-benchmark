cmake_minimum_required(VERSION 3.8)
project(task_009_nodelet_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Plus component (shared library)
add_library(plus_component SHARED src/plus.cpp)
target_include_directories(plus_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(plus_component rclcpp rclcpp_components std_msgs)
rclcpp_components_register_node(plus_component
  PLUGIN "test_nodelet::Plus"
  EXECUTABLE plus_node
)

# NodehandleTest component (shared library)
add_library(nodehandle_test_component SHARED src/nodehandles.cpp)
target_include_directories(nodehandle_test_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(nodehandle_test_component rclcpp rclcpp_components std_msgs builtin_interfaces)
rclcpp_components_register_node(nodehandle_test_component
  PLUGIN "test_nodelet::NodehandleTest"
  EXECUTABLE nodehandle_test_node
)

install(TARGETS plus_component nodehandle_test_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()