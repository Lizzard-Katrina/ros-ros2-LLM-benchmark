# Task 3: NRP Experimental FSM

## Task Description
This task requires translating a ROS1 NRP Experimental FSM (SMACH) node to ROS2. The flow is:

**Experiment Input → SMACH State Machine → Simulation Commands → Feedback**

LLM is expected to fill in the TODO variants while preserving all ROS topics, parameters, and class/function signatures.


## TODO Variants
- **variant_01_state_logic_missing.py**: Fill in missing SMACH state logic.
- **variant_02_sim_command_missing.py**: Fill in missing simulation environment commands.
- **variant_03_feedback_processing_missing.py**: Fill in missing feedback processing logic.

## Expected Outcome
- Each variant should run without syntax errors.
- States transition correctly.
- Simulation commands are correctly sent to environment.
- Feedback is correctly processed to update SMACH userdata.

