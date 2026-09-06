cmake_minimum_required(VERSION 3.8)
project(task_007_aws_les)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/KeyValue.msg"
  "srv/AudioTextConversation.srv"
  DEPENDENCIES std_msgs
)

# We need to get the typesupport target for our own interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Library for lex_node
add_library(lex_node_lib
  src/lex_node.cpp
)
target_include_directories(lex_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(lex_node_lib rclcpp std_msgs)
target_link_libraries(lex_node_lib "${cpp_typesupport_target}")

# Main executable
add_executable(lex_node_main src/main.cpp)
target_include_directories(lex_node_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(lex_node_main rclcpp std_msgs)
target_link_libraries(lex_node_main lex_node_lib "${cpp_typesupport_target}")

install(TARGETS lex_node_main
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS lex_node_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_runtime_ros2
    ${CMAKE_CURRENT_SOURCE_DIR}/test_runtime_ros2.py
    TIMEOUT 60
  )
endif()

ament_package()