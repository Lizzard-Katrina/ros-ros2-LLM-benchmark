# Simulation Integration Task 001 - Classic Smoke Test

## Task Overview
This task verifies a basic simulation integration pipeline:
- Spawn a robot URDF in Gazebo.
- Connect simulation plugins (robot_state_publisher, joint_state_publisher, RViz visualization).
- Publish `/robot_description` topic and visualize the robot in RViz.

The focus is on **simulation integration**, ensuring the URDF/XACRO can be loaded, Gazebo spawns the robot, and ROS topics and visualization are functional.

## Files to Edit / TODO
- **urdf_tutorial/urdf/08-macroed.urdf.xacro**
  ```xml
  <!-- TODO: Define the robot's base_link including visual, collision, and inertial properties -->
 ```

## Expected Outcome

Robot successfully spawns in Gazebo using display.launch.

/robot_description topic is published correctly.

Robot model appears in RViz as expected.

Joint states can be manipulated with GUI or non-GUI joint_state_publisher.

## How to Run

Build Docker container:
```
docker build -t sim_integration_001 .
```

Start container and source workspace:
```
docker run -it sim_integration_001
source /root/catkin_ws/devel/setup.bash
```

Launch simulation:
```
roslaunch urdf_tutorial display.launch
```

Verify Gazebo and RViz outputs.
