cmake_minimum_required(VERSION 3.8)
project(task_009_urdf)

find_package(ament_cmake REQUIRED)

install(FILES
  arm_urdf.urdf
  manipulator.srdf
  joint_limits.yaml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()