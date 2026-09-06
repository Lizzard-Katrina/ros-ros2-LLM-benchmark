cmake_minimum_required(VERSION 3.8)
project(task_001_laser_obstacle_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(rcl_interfaces REQUIRED)

# filters and diagnostic_updater are optional at cmake level
find_package(filters QUIET)
find_package(diagnostic_updater QUIET)

set(NODE_DEPS
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2
  tf2_ros
  message_filters
  rcl_interfaces
)

add_library(scan_to_scan_filter_chain_component SHARED
  scan_to_scan_filter_chain.cpp
)
target_include_directories(scan_to_scan_filter_chain_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

if(filters_FOUND)
  list(APPEND NODE_DEPS filters)
  target_compile_definitions(scan_to_scan_filter_chain_component PRIVATE HAS_FILTERS=1)
else()
  message(WARNING "filters package not found – filter chain will be a pass-through")
endif()

if(diagnostic_updater_FOUND)
  list(APPEND NODE_DEPS diagnostic_updater)
  target_compile_definitions(scan_to_scan_filter_chain_component PRIVATE HAS_DIAGNOSTIC_UPDATER=1)
else()
  message(WARNING "diagnostic_updater package not found – diagnostics disabled")
endif()

ament_target_dependencies(scan_to_scan_filter_chain_component ${NODE_DEPS})

rclcpp_components_register_node(scan_to_scan_filter_chain_component
  PLUGIN "ScanToScanFilterChain"
  EXECUTABLE scan_to_scan_filter_chain_node
)

install(TARGETS scan_to_scan_filter_chain_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()