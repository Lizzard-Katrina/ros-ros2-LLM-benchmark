cmake_minimum_required(VERSION 3.20)
project(turtlesim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# TODO [Hole 3]: Complete the build configuration for the turtlesim package.
# Your script must resolve all middleware, message, and GUI dependencies 
# required for the simulator and its tutorials. Configure the build 
# pipeline to ensure all C++ targets are correctly linked to their 
# respective interfaces and system libraries. Setup the necessary 
# installation rules for a standard ROS 2 environment.
# END OF TODO
