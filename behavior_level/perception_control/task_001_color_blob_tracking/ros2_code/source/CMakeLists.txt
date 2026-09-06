cmake_minimum_required(VERSION 3.8)
project(task_001_color_blob_tracking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DriveToTarget.srv"
  DEPENDENCIES std_msgs
)

# We need to use the generated typesupport from this package
# For rclcpp targets that use the generated interfaces from the same package,
# we use rosidl_get_typesupport_target
add_executable(process_image src/process_image.cpp)
ament_target_dependencies(process_image rclcpp sensor_msgs geometry_msgs)
rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(process_image "${typesupport_target}")

add_executable(drive_bot src/drive_bot.cpp)
ament_target_dependencies(drive_bot rclcpp geometry_msgs)
rosidl_get_typesupport_target(typesupport_target2 ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(drive_bot "${typesupport_target2}")

install(TARGETS
  process_image
  drive_bot
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()