# Simulation Integration 002: Gazebo ↔ ROS Demo Scaffold

## Purpose
This task tests the integration between ROS and Gazebo. Students are expected to complete the missing parts of a launch file and a Gazebo world plugin to enable simulation, spawn robots, and ensure ROS ↔ Gazebo communication works correctly.

## Source repository
[ros-simulation/gazebo_ros_demos](https://github.com/ros-simulation/gazebo_ros_demos)

## Relevant Source Directories
- Launch file: `gazebo_tutorials/launch/hello.launch`
- Plugin file: `gazebo_tutorials/src/simple_world_plugin.cpp`

## Expected Outcome

The launch file successfully starts Gazebo with the hello.world environment.

The ROS node and plugin are correctly loaded and initialized.

Plugin outputs "Hello World!" to ROS log.

Students are able to extend the plugin to interact with the simulation environment (e.g., publishing/subscribing to topics, spawning objects).
