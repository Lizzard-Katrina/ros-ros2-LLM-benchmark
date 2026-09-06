cmake_minimum_required(VERSION 3.8)
project(task_010_reach_study)

find_package(ament_cmake REQUIRED)

# These source files depend on reach_ros headers (not available as a standard
# ros-humble package), so we only install them for inspection / static analysis.
install(FILES
  moveit_ik_solver.cpp
  manipulability_moveit.cpp
  transformed_point_cloud_target_pose_generator.cpp
  DESTINATION share/${PROJECT_NAME}
)

ament_package()