# Task 005: Demo Manipulator Arms in ROS1 (with multiple arms and demos)

**Source repository:** [Interbotix ROS Manipulators](https://github.com/Interbotix/interbotix_ros_manipulators)

**Task overview:**  
This task requires students to correctly select and execute a manipulator demo in ROS1. The focus is **integration-level correctness**, i.e., selecting robot models, launch modes, and interfaces, without requiring knowledge of detailed trajectories or planning algorithms. Students must configure multiple launch files and choose the correct demo sequence.

---

## Subtasks

### Subtask 1: Select the demo entrypoint
- **File:** `ros1_code/demo_launcher.py`
- **Source snippet:**
```python
# examples/python_demos/joint_trajectory_control.py

def main():

    trajectory = [
        {0.0: [0.0,  0.0, 0.0, 0.0, 0.0, 0.0]},
        {2.0: [0.0,  0.0, 0.0, 0.0, 0.5, 0.0]},
        {4.0: [0.5,  0.0, 0.0, 0.0, 0.5, 0.0]},
        {6.0: [-0.5, 0.0, 0.0, 0.0, 0.5, 0.0]}
    ]

    bot = InterbotixManipulatorXS("wx250s", "arm", "gripper")
    bot.arm.go_to_home_pose()
    bot.dxl.robot_write_trajectory("group", "arm", "position", trajectory)
# Task: Choose which demo entrypoint to run and call it.

###Subtask 2: Configure control launch

File: ros1_code/launch_control_config.launch

Source snippet: xsarm_control.launch
###Subtask 3: Configure MoveIt execution mode

File: ros1_code/launch_moveit_config.launch

Source snippet: xsarm_moveit.launch
###Subtask 4: Configure interface

File: ros1_code/launch_interface_config.launch

Source snippet: interface.launch

#Expected outcome

After selecting the correct demo, control configuration, execution mode, and interface:

ROS nodes launch without errors.

Robot arm executes the demo trajectory correctly (or fake execution if selected).

Students demonstrate correct integration of multiple launch files.

