# Task006 — Airbus SCXML-based Smart State-Machine (SSM) — ROS1 scaffold

## Purpose
This task provides a compact ROS1 scaffold derived from Airbus's SCXML SSM (airbus_ssm_core). It focuses on the runtime parts that are essential to verify whether an LLM can translate and reconstruct state-machine behavior from ROS1 to ROS2:

- SCXML → internal graph construction
- Event dispatch / mapping to transitions
- Transition evaluation (conditions, userdata updates, state outcomes)

## Directory layout
See the repository tree. Key files:
- `ros1_code/src/ssm_core/ssm_runtime.py` — SSM runtime (original & variants)
- `ros1_code/src/ssm_core/scxml_loader.py` — minimal SCXML loader
- `ros1_code/src/states/basic_states.py` — example State classes used in SCXML
- `ros1_code/src/variants/` — contains 3 variants. Each variant is a complete module (full context) with exactly one `# TODO` … `# END` block.

## Variants (single TODO each)
1. `variant_01_graph_construction_missing.py` — removed SCXML parsing → graph construction block. LLM must rebuild how SCXML maps to internal nodes/transitions.
2. `variant_02_event_dispatch_missing.py` — removed event dispatch loop / handler. LLM must reconstruct thread-safe event consumption and how events cause state activation/transition.
3. `variant_03_transition_eval_missing.py` — removed transition condition evaluation & outcome handling. LLM must restore condition checks, userdata evaluation, and proper outcome application.

## Validation hints
- A minimal SCXML example `scxml_examples/simple_demo.scxml` is included. It defines a small behavior: `stateA` → `stateB` → `final`.
- Validation approach (per variant):
  - Use the **original** module to produce expected logs for the example SCXML (e.g., sequence of state names and final outcome).
  - Replace runtime with variant (or run variant directly if it's a full module) and have LLM fill the TODO, then run the same demo.
  - Compare logs / final userdata / final outcome with original.
- A small test script `ros1_code/src/tests/validate_variant.sh` is included as a smoke test skeleton.

## How to use
1. Place `ros1_code` into a catkin package (e.g., `airbus_ssm_task006_demo`) so `rosrun`/`roslaunch` can find modules.
2. `roscore`
3. Launch demo: `roslaunch launch/ssm_demo.launch` (adjust pkg/type if needed).
4. For variant validation: run `rosrun <pkg> ssm_runtime.py` (or the variant module) and send events via the test script.

## Notes
- Variants are adapted from the original Airbus code with minimal changes to run in this compact scaffold; original copies are in `original/` for direct reference.
- All TODO blocks are in English and contain short hints. They are intentionally non-trivial to encourage realistic LLM reconstruction.
