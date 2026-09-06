cmake_minimum_required(VERSION 3.8)
project(task_008_exploration_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Try to find ros2_control packages; if not available, use bundled stubs
find_package(hardware_interface QUIET)
find_package(pluginlib QUIET)
find_package(angles QUIET)

if(hardware_interface_FOUND AND pluginlib_FOUND AND angles_FOUND)
  message(STATUS "Using system hardware_interface, pluginlib, angles")
  set(USE_SYSTEM_HW_INTERFACE TRUE)
else()
  message(STATUS "hardware_interface/pluginlib/angles not found; building with bundled stubs")
  set(USE_SYSTEM_HW_INTERFACE FALSE)
endif()

add_library(robot_hardware_interface SHARED
  robot_hardware_interface_node.cpp
)

target_include_directories(robot_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(USE_SYSTEM_HW_INTERFACE)
  ament_target_dependencies(robot_hardware_interface
    rclcpp
    hardware_interface
    pluginlib
    angles
  )
else()
  target_compile_definitions(robot_hardware_interface PRIVATE USE_STUB_HW_INTERFACE)
  ament_target_dependencies(robot_hardware_interface
    rclcpp
  )
endif()

install(TARGETS robot_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES controllers.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

install(FILES task_008_exploration_mapping_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(robot_hardware_interface)
ament_export_dependencies(rclcpp)

ament_package()