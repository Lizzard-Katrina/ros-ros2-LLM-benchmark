# ROS1 Manipulator Benchmark Scaffold

This repository contains a benchmark scaffold for a generic 6-DOF manipulator with gripper, built using:

- ROS1
- MoveIt
- Gazebo simulation

The scaffold is extracted and simplified from [ros_moveit_gazebo_ws](https://github.com/lFatality/ros_moveit_gazebo_ws).  
The purpose of this benchmark is to test LLMs on reasoning about robotic control logic, MoveIt configurations, and URDF definitions.


---

## Scaffold Design

1. **arm_urdf.urdf**  
   - Contains the robot links and basic joint structure.  
   - Core joint logic for `joint2~joint6` and fingers is removed (`TODO`) to test LLM completion of URDF limits, dynamics, and transmissions.

2. **demo.launch**  
   - Loads robot description and starts MoveIt demo nodes.  
   - Core MoveIt launch logic (move_group, RViz, fake execution, database) is removed (`TODO`) for LLM reasoning tests.

3. **ros_controllers.yaml**  
   - Defines hardware interface, joint list, and controller list.  
   - Controller gains and PID settings are removed (`TODO`) to challenge LLMs to fill in correct logic.

---

## How to Use

1. Clone this repository.  
2. Place it in your ROS workspace (`catkin_ws/src`).  
3. Run `catkin_make` or `catkin build`.  
4. Inspect the TODOs in each file.  
5. Complete the missing logic to make the robot simulation functional.

---

## Notes

- This scaffold is **not meant for running directly**; it is intended for benchmarking and testing LLM completion of robotic control logic.  
- The TODOs highlight challenging parts of the code that require domain-specific reasoning.  
- The original repository [ros_moveit_gazebo_ws](https://github.com/lFatality/ros_moveit_gazebo_ws) can be used as reference for validation.


