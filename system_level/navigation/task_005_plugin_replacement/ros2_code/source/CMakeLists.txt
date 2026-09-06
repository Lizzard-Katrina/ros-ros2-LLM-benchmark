cmake_minimum_required(VERSION 3.8)
project(task_005_plugin_replacement)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)

# Note: This is a reference migration package. The actual navfn library
# and nav_core interface are provided as header/source files for validation.
# We do not build them into a real shared library here because the full
# navfn algorithm source (navfn.cpp) and costmap dependencies would be
# needed. Instead we ensure the build system structure is correct.

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install plugin description file
install(FILES bgp_plugin.xml DESTINATION share/${PROJECT_NAME})

ament_package()