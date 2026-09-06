cmake_minimum_required(VERSION 3.8)
project(task_003_mp3_db_service)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MP3InventoryService.srv"
)

# Install Python scripts
install(PROGRAMS
  mp3_controller.py
  mp3_server.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()