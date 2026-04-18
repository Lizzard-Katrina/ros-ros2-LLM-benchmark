cmake_minimum_required(VERSION 3.0.2)
project(actionlib)

find_package(catkin REQUIRED COMPONENTS actionlib_msgs message_generation roscpp std_msgs)
find_package(Boost REQUIRED COMPONENTS thread)

# TODO: Complete the ROS 2 ament_cmake build configuration.
# 1. Find packages: rclcpp, rclcpp_action, rosidl_default_generators.
# 2. Use rosidl_generate_interfaces to generate action code for:
#    Test.action, TestRequest.action, and TwoInts.action.
# 3. Create the 'actionlib' library/executable and link it using ament_target_dependencies.
# 4. Export dependencies and use ament_package().
#END OF TODO
