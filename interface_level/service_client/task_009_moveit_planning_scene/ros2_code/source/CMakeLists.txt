cmake_minimum_required(VERSION 3.8)
project(task_009_moveit_planning_scene)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CollisionObject.msg"
  "msg/AttachedCollisionObject.msg"
  "msg/RobotState.msg"
  "msg/PlanningSceneWorld.msg"
  "msg/AllowedCollisionEntry.msg"
  "msg/AllowedCollisionMatrix.msg"
  "msg/LinkPadding.msg"
  "msg/LinkScale.msg"
  "msg/ObjectColor.msg"
  "msg/PlanningScene.msg"
  DEPENDENCIES geometry_msgs shape_msgs std_msgs sensor_msgs octomap_msgs
)

# Build the executable using the generated messages
add_executable(apply_planning apply_planning.cpp)
ament_target_dependencies(apply_planning
  rclcpp
  geometry_msgs
  shape_msgs
  std_msgs
  sensor_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(apply_planning "${cpp_typesupport_target}")

install(TARGETS apply_planning
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()