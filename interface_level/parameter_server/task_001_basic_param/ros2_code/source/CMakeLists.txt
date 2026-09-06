cmake_minimum_required(VERSION 3.8)
project(task_001_basic_param)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetCachedParam.srv"
)

add_library(param_cache
  param.cpp
)
target_include_directories(param_cache PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(param_cache PUBLIC cxx_std_17)
ament_target_dependencies(param_cache
  rclcpp
  rcl_interfaces
)

add_executable(param_cache_node
  src/param_cache_node.cpp
)
target_include_directories(param_cache_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(param_cache_node PUBLIC cxx_std_17)
ament_target_dependencies(param_cache_node
  rclcpp
  rcl_interfaces
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(param_cache_node
  param_cache
  "${cpp_typesupport_target}"
)

install(TARGETS
  param_cache
  param_cache_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(param_cache)
ament_export_dependencies(rclcpp rcl_interfaces rosidl_default_runtime)

ament_package()