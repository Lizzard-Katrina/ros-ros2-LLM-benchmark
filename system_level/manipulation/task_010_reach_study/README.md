# Task 010: Manipulation Integration – Reach Study

## Task Overview
This task focuses on **integration-level correctness** in manipulation tasks using the `reach_ros` package.  
The goal is to evaluate a robot manipulator's **inverse kinematics (IK)** and **manipulability** computation using MoveIt and the planning scene.  
The LLM is required to fill in the **core algorithmic logic** while preserving the surrounding ROS structure and interfaces.

**Source:** [ros-industrial/reach_ros](https://github.com/ros-industrial/reach_ros)  

### Key Concepts
- Integration-level correctness: ensures that the IK solver interacts correctly with the planning scene and collision checking, and that manipulability metrics are computed correctly.
- The task preserves the **original class and function signatures**, ROS topic names, parameter structure, and internal data structures.
- Only the **core algorithm blocks** are removed (marked with `TODO`) for the LLM to fill.

##TODO Details
###1. IK Solver TODO

File: ros1_code/ik_solveIK_todo.cpp

Original Source File: src/ik/moveit_ik_solver.cpp

Function/Method: MoveItIKSolver::solveIK

Description: Fill in the algorithm that:

Initializes a RobotState with the robot model

Extracts joint subset from the seed map

Sets joint group positions

Calls the MoveIt IK solver with the isIKSolutionValid callback

Returns valid solutions in the expected reach format

Expected Outcome: LLM should implement a working IK solver that respects the planning scene, collision objects, distance thresholds, and returns joint solutions.

### 2. Manipulability TODO

File: ros1_code/manipulability_score_todo.cpp

Original Source File: src/evaluation/manipulability_moveit.cpp

Function/Method: ManipulabilityMoveIt::calculateScore(const std::map<std::string, double>& pose)

Description: Fill in the algorithm that:

Constructs RobotState from the robot model

Extracts active joint values from the input pose map

Updates robot state

Computes the Jacobian for the joint model group

Applies optional row subset

Performs SVD and computes manipulability score

Expected Outcome: LLM should compute manipulability correctly based on singular values, respecting any Jacobian row subsets and excluded links
