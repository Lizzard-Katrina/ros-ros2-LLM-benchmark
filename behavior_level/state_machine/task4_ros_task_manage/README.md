# Task 4: ROS Task Manager

## Task Description
Translate ROS1 Task Manager code to ROS2. Flow:

**Task Input → SMACH Planner → Actionlib Commands → Robot Actuators → Feedback**

LLM is expected to fill in the TODO variants while preserving all ROS topics, parameters, and class/function signatures.


## TODO Variants
- **variant_01_smach_planner_missing.py**: Fill in missing SMACH state planning logic.
- **variant_02_action_call_missing.py**: Fill in missing actionlib command sending.
- **variant_03_feedback_processing_missing.py**: Fill in missing feedback processing.

## Expected Outcome
- Each variant runs without syntax errors.
- SMACH state transitions behave correctly (variant_01).
- Actionlib commands are correctly sent and result handled (variant_02).
- Feedback is correctly processed and SMACH userdata updated (variant_03).

## Launch
```bash
roslaunch ros1_code/launch/task_manager.launch
