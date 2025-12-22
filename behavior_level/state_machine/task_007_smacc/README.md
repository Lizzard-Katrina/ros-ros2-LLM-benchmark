# Task 007 — SMACC (State Machine Asynchronous C++) — ROS1 scaffold

## Purpose
This task provides a compact ROS1 C++ scaffold inspired by SMACC (robosoft-ai/SMACC) to evaluate LLMs on translating hierarchical asynchronous state machines. It focuses on three core runtime aspects:

- State execution (entry/exit and state lifecycle)
- Transition guard evaluation (checking conditions for transitions)
- Event queue processing and dispatch

Each variant removes a single core logic block (taken from the original runtime), preserving all surrounding code and interfaces. The LLM must reconstruct the missing logic so that the translated ROS2 C++ implementation behaves like the original.


## Variants
1. `variant_01_state_exec_missing.cpp` — the runtime's state execution (calling state entry/exit, state lifecycle management) is removed.
2. `variant_02_transition_guard_missing.cpp` — the logic for evaluating transition guards (conditions) is removed.
3. `variant_03_event_dispatch_missing.cpp` — the event queue consumption and dispatch loop is removed.

## Expected Outcomes / Validation
- After LLM fills the TODO and the code is translated/compiled to ROS2, behavior should match original:
  - Running the demo should print or log the same state entry sequence.
  - Transitions that require guard conditions should only occur when conditions are satisfied.
  - Events posted should be consumed and result in appropriate transitions.
- A small test harness and example will be provided to check:
  - state sequence logs
  - active state name after events
  - guard-dependent transitions

## How to use
1. Place `ros1_code` into a catkin package (e.g., `smacc_task007_demo`) with dependencies (roscpp).
2. `catkin_make` / `catkin build`
3. `roscore` then:
roslaunch launch/smacc_demo.launch

4. Replace original runtime with a variant (or run variant directly) and let LLM fill TODO.

## Notes
- All variants are based on the style and structure of the real SMACC C++ code. Original, unmodified files are included under `ros1_code/src/original/` for comparison.
- TODO blocks are in C++ comment form `// TODO` ... `// END`.
