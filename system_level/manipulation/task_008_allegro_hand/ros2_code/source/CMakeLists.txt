cmake_minimum_required(VERSION 3.8)
project(task_008_allegro_hand)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-reorder)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Build the driver library (no real CAN hardware needed)
add_library(allegro_hand_driver SHARED
  src/AllegroHandDrv.cpp
)
target_include_directories(allegro_hand_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(allegro_hand_driver rclcpp)

# Build the controller node
add_executable(allegro_node
  src/allegro_node.cpp
)
target_include_directories(allegro_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(allegro_node allegro_hand_driver)
ament_target_dependencies(allegro_node rclcpp sensor_msgs std_msgs)

install(TARGETS allegro_hand_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS allegro_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_runtime_ros2
    ${CMAKE_CURRENT_SOURCE_DIR}/test_runtime_ros2.py
    TIMEOUT 30
  )
endif()

ament_package()