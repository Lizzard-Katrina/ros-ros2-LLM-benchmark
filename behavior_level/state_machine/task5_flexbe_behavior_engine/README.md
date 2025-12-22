# Task 005 — FlexBE Behavior Engine (ROS1) — Benchmark Scaffold

## Purpose
This task provides a ROS1 scaffold derived from the FlexBE behavior engine (FlexBE/flexbe_behavior_engine) to test LLMs translating ROS1 → ROS2. Each variant removes a specific core logic block (taken from the original code), and the LLM must restore it. After translation to ROS2, the behavior should match the original.

## Flow
Input → BehaviorEngine → State execution loop → Event handling → Feedback processing → Actuation

## Directory layout
See the directory tree in the repo root. Key locations:
- `ros1_code/src/flexbe_core/behavior_engine.py` — full module (original)
- `ros1_code/src/flexbe_states/` — two example states used by the engine
- `ros1_code/src/original/` — contains `__ORIGINAL.py` copies used for reference
- `ros1_code/src/variants/` — contains 3 variant files; each is a complete module with one `# TODO` … `# END` block (single logical block removed)

## Variants (single TODO block each)
1. **variant_01_state_execution_missing.py** — the engine `_execute` state execution loop is removed. LLM must restore how the engine iterates active states, calls state `execute`, handles state completion, and advances the behavior.
2. **variant_02_event_handling_missing.py** — the `_handle_event` routine is removed. LLM must restore how events (external triggers, timers) map to state transitions and userdata updates.
3. **variant_03_feedback_processing_missing.py** — the `_process_state_feedback` logic inside the engine/state glue is removed. LLM must restore processing that maps state feedback (e.g. success/failure/partial progress) to behavior outcomes and userdata updates.

## Validation hints
- Use the `original/` files as ground truth. After LLM fills the TODO and you translate to ROS2, verify that:
  - The execution order of states matches the original run for a deterministic demo (the demo included prints/logs).
  - Events cause the same transitions.
  - Feedback leads to the same success/failure decisions and userdata values.
- The `ros1_code/src/tests/run_smoke_tests.sh` script contains smoke tests for running the engine and capturing output logs. The script is a skeleton you can extend.

## How to use
1. Copy `ros1_code` into a catkin package (e.g., `flexbe_task5_demo`) with required dependencies and messages.
2. `roscore`
3. Launch the demo:
4. For a variant, run the corresponding module in place of the original (or replace the original behavior_engine.py with the variant).

## Notes
- All TODO blocks are extracted from the original engine, adapted only minimally to make the file runnable in isolation. Each variant file contains a comment header indicating which original code region was removed.
- Comments/hints inside each TODO are intentionally small — enough to orient the LLM but not to give away the full implementation.
