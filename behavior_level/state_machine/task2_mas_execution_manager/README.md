# Task 2 — MAS Execution Manager (ROS1) — Benchmark Scaffold

## Overview
This task is a simplified MAS Execution Manager that uses SMACH and actionlib to execute high-level tasks. It is used in the benchmark to evaluate LLMs' ability to translate or reconstruct ROS code that involves state machines and action interaction.

Flow:
Sensor/Task Input -> Execution Manager (SMACH) -> Actionlib Client -> Action Server -> Robot Actuators


## What is included
- `ros1_code/launch/mas_exec_manager.launch` — launch file to start the manager.
- `ros1_code/nodes/execution_manager.py` — node that builds and runs the SMACH state machine (original version).
- `ros1_code/src/mas_exec_manager/execution_manager_core.py` — original state definitions (LoadTask, ExecuteTask, CheckResult, Recovery, Finished).
- `ros1_code/src/mas_exec_manager/action_client_wrapper.py` — original actionlib client wrapper.

## Purpose of Originals vs Variants
- **Originals**: Full working reference implementations (provided here so you can evaluate the complexity and correctness of the removed blocks).
- **Variants**: Files where a single core logic block is removed and replaced by a `# TODO ... # END` block. Each variant file contains exactly one TODO block. Variants are under `ros1_code/src/mas_exec_manager/variants/` (sent separately).

## Evaluation targets
LLMs will be evaluated on their ability to:
- Restore SMACH flow and transitions (including failure/retry/recovery paths).
- Restore actionlib goal send/wait/result/cancel flow.
- Maintain correct usage of userdata and rospy params where applicable.

## How to run (local, ROS1)
1. Place this directory under a catkin package (package name `mas_exec_manager`), or copy files into an existing package with same message/action dependencies.
2. `roscore` in one terminal.
3. Launch the manager:
   ```bash
   roslaunch ros1_code/launch/mas_exec_manager.launch


