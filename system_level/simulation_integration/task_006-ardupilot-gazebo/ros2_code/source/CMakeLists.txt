cmake_minimum_required(VERSION 3.8)
project(task_006-ardupilot-gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# We build a simple static library from the source files for verification.
# The actual Gazebo plugin loading is not needed for the test;
# we just need the code to compile and be inspectable.

# Build a helper library from SocketUDP (no Gazebo deps needed)
add_library(socket_udp STATIC
  SocketUDP.cc
)
target_include_directories(socket_udp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

# Install source files so the test can find them
install(FILES
  ArduPilotPlugin.cc
  SocketUDP.cc
  ArduPilotPlugin.hh
  SocketUDP.hh
  Util.hh
  Util.cc
  DESTINATION share/${PROJECT_NAME}
)

# Install the socket_udp library and a small test executable
add_executable(test_socket_udp test_socket_udp_main.cc)
target_link_libraries(test_socket_udp socket_udp)
target_include_directories(test_socket_udp PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

install(TARGETS test_socket_udp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()