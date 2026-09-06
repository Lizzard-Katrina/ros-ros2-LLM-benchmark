cmake_minimum_required(VERSION 3.8)
project(task_010_pick_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# MoveIt dependencies are optional – the full executable can only be built
# when moveit_ros_planning_interface, moveit_core and tf2_geometry_msgs are
# available.  When they are not (e.g. CI / minimal Docker images) we still
# install the source so that tests can validate the translation.
find_package(moveit_ros_planning_interface QUIET)
find_package(moveit_core QUIET)
find_package(tf2_geometry_msgs QUIET)

if(moveit_ros_planning_interface_FOUND AND moveit_core_FOUND AND tf2_geometry_msgs_FOUND)
  add_executable(pick_place src/pick_place.cpp)
  ament_target_dependencies(pick_place
    rclcpp
    moveit_ros_planning_interface
    moveit_core
    trajectory_msgs
    geometry_msgs
    tf2_geometry_msgs
  )
  install(TARGETS pick_place
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(STATUS "MoveIt not found – skipping pick_place executable build, installing source only.")
endif()

# Always install the source file so runtime tests can inspect it
install(FILES src/pick_place.cpp
  DESTINATION share/${PROJECT_NAME}/src
)

ament_package()