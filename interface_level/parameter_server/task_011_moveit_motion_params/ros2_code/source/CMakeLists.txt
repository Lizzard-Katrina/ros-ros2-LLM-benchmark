cmake_minimum_required(VERSION 3.8)
project(task_011_moveit_motion_params)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# MoveIt packages are optional – the tutorial needs them at runtime but
# we must still produce a buildable package in environments where they
# are not installed.
find_package(moveit_core QUIET)
find_package(moveit_ros_planning QUIET)
find_package(moveit_ros_planning_interface QUIET)
find_package(moveit_visual_tools QUIET)

set(_DEPS rclcpp geometry_msgs)
set(_DEFS "")

if(moveit_core_FOUND AND moveit_ros_planning_interface_FOUND)
  list(APPEND _DEPS moveit_core moveit_ros_planning_interface)
  if(moveit_ros_planning_FOUND)
    list(APPEND _DEPS moveit_ros_planning)
  endif()
  if(moveit_visual_tools_FOUND)
    list(APPEND _DEPS moveit_visual_tools)
  else()
    list(APPEND _DEFS NO_MOVEIT_VISUAL_TOOLS)
  endif()
else()
  # Build a minimal stub so the executable still gets installed
  list(APPEND _DEFS NO_MOVEIT NO_MOVEIT_VISUAL_TOOLS)
endif()

add_executable(moveit_cpp_tutorial moveit_cpp_tutorial.cpp)
target_compile_definitions(moveit_cpp_tutorial PRIVATE ${_DEFS})
ament_target_dependencies(moveit_cpp_tutorial ${_DEPS})

install(TARGETS moveit_cpp_tutorial
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()