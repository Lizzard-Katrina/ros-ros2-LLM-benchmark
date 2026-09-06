cmake_minimum_required(VERSION 3.8)
project(task_006_exploration_mapping)

find_package(ament_cmake REQUIRED)

install(FILES
  mapping.launch
  costmap_common_params.yaml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()