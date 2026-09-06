cmake_minimum_required(VERSION 3.8)
project(task_002_gazebo_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Gazebo is optional – only build the plugin when the SDK is present
find_package(gazebo_dev QUIET)
find_package(gazebo_ros QUIET)

if(gazebo_dev_FOUND AND gazebo_ros_FOUND)
  message(STATUS "Gazebo found – building simple_world_plugin")

  link_directories(${gazebo_LIBRARY_DIRS})

  add_library(simple_world_plugin SHARED
    src/simple_world_plugin.cpp
  )

  target_include_directories(simple_world_plugin PUBLIC
    ${gazebo_INCLUDE_DIRS}
  )

  ament_target_dependencies(simple_world_plugin
    rclcpp
    gazebo_ros
    gazebo_dev
  )

  target_link_libraries(simple_world_plugin
    ${gazebo_LIBRARIES}
  )

  install(TARGETS simple_world_plugin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
else()
  message(STATUS "Gazebo NOT found – skipping simple_world_plugin build (source kept for reference)")
endif()

# Always install launch and world files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# Install the source file so tests can find it
install(DIRECTORY src/
  DESTINATION share/${PROJECT_NAME}/src
)

ament_package()