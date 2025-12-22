# Task 008A — Robot Statemachine (RSM) scaffold

## Purpose
Advanced scaffold to test LLM translation & reconstruction for state-machines (from MarcoStb1993/robot_statemachine). Variants remove core logic; LLM must restore it preserving interface and runtime semantics.

## Included files
- `ros1_code/src/original/` — ORIGINAL modules (state, transitions, state_machine, robot_node) used as ground truth.
- `rsm_core/*` — variant files (state.py, transitions.py, state_machine.py) with TODO blocks to fill.
- `rsm_nodes/robot_node.py` — variant node containing TODO to instantiate states/transitions.

## Expected outcome (per-variant)
When a variant's TODO is correctly filled (and the module is run with the provided demo parameters), the runtime must:

1. **State lifecycle**  
   - On starting the state machine, the active state's `on_enter` is called and logs a message containing the state name and start time.  
   - When transitioning away, `on_exit` is called and it logs the duration the state was active.

2. **Transition evaluation**  
   - `Transition.evaluate(state_machine)` returns `True` only if `condition` is `None` (treated as unconditional) or `condition(state_machine)` returns truthy.  
   - Conditions must be executed in a safe manner (try/except) and must not modify unrelated state_machine fields.

3. **Run & transition semantics**  
   - `StateMachine.run()` must:
     - call `current_state.execute()` each loop iteration,
     - read the outcome string and look up transitions for the current state name,
     - evaluate transitions in insertion order,
     - on first matching transition, call `current_state.on_exit()`, switch the active state to the transition target, call `new_state.on_enter()` and continue.

4. **RobotNode construction**  
   - `RobotNode` must instantiate state objects exactly matching the original demo (state classes and outcomes), add transitions appropriately, set the start state, and call `sm.run()`.

## Validation checklist (how you will check LLM output)
- Run ORIGINAL module and capture logs for a deterministic demo (set userdata so transitions are deterministic).
- Run LLM-filled variant module with same userdata and compare:
  - Sequence of "[StateMachine] Transition" log lines (same source & target order).
  - Active state name after each event or after N iterations (should match original).
  - Presence of `on_enter`/`on_exit` logs and approximate durations (>0).
- For `Transition.evaluate`, add a test condition where the condition reads `sm.userdata.counter` and only fires if `counter >= 1`. The variant implementation should match that behavior.

## How to run (quick)
1. Put `ros1_code/src` into a reachable PYTHONPATH or catkin package.  
2. Start ROS: `roscore &`.  
3. Run original demo: `python3 ros1_code/src/original/robot_node__ORIGINAL.py`. Capture logs.  
4. Replace original with variant file (fill TODO), run variant, compare logs.

