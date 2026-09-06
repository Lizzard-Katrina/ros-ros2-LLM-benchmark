cmake_minimum_required(VERSION 3.8)
project(task_002_camera_depth_reach_target)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# moveit_ros_planning_interface is optional; only build the full executable
# when it is available (it requires a full MoveIt2 installation).
find_package(moveit_ros_planning_interface QUIET)

if(moveit_ros_planning_interface_FOUND)
  add_executable(depth_reach src/depth_reach.cpp)
  ament_target_dependencies(depth_reach
    rclcpp
    moveit_ros_planning_interface
    moveit_msgs
    geometry_msgs
    trajectory_msgs
    shape_msgs
    tf2_geometry_msgs
  )
  install(TARGETS depth_reach
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(STATUS "moveit_ros_planning_interface not found -- building stub executable")
  add_executable(depth_reach src/depth_reach_stub.cpp)
  ament_target_dependencies(depth_reach
    rclcpp
    moveit_msgs
    geometry_msgs
    trajectory_msgs
    shape_msgs
    tf2_geometry_msgs
  )
  install(TARGETS depth_reach
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# Install the real source so tests can inspect it
install(FILES src/depth_reach.cpp
  DESTINATION share/${PROJECT_NAME}/src
)

ament_package()