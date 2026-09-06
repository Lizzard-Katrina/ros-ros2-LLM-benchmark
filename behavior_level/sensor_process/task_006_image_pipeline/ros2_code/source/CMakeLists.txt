cmake_minimum_required(VERSION 3.8)
project(task_006_image_pipeline)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_geometry REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)

add_library(point_cloud_xyzrgb_radial SHARED
  src/point_cloud_xyzrgb_radial.cpp
)

target_include_directories(point_cloud_xyzrgb_radial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(point_cloud_xyzrgb_radial
  rclcpp
  rclcpp_components
  sensor_msgs
  image_transport
  image_geometry
  cv_bridge
  message_filters
  OpenCV
)

rclcpp_components_register_node(point_cloud_xyzrgb_radial
  PLUGIN "depth_image_proc::PointCloudXyzrgbRadialNode"
  EXECUTABLE point_cloud_xyzrgb_radial_node
)

install(TARGETS point_cloud_xyzrgb_radial
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES point_cloud_xyzrgb_radial.cpp
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_runtime_ros2 test_runtime_ros2.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()