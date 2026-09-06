cmake_minimum_required(VERSION 3.8)
project(task_007_smacc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# We build the translated source as a library so it can be verified.
# Since the real SMACC headers don't exist in this minimal package we
# supply a local stub header that satisfies compilation.
add_library(smacc_async_behavior SHARED
  smacc_client_async_behavior.cpp
)
target_include_directories(smacc_async_behavior PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(smacc_async_behavior rclcpp)

install(TARGETS smacc_async_behavior
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

ament_package()