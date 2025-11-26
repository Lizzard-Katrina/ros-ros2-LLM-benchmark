# Task 001 — Robot Calibration Action

## Description
The task provides an Action definition for robot calibration and two high-difficulty stubs:
- `calibration_action_server_stub.py` (server, heavily blanked)
- `calibration_action_client_stub.py` (client, heavily blanked)

The .action file is complete. The server/client stubs intentionally remove nearly the entire core logic:
- imports for action types
- server name and start call
- feedback/result field names
- preemption handling
- loop control and sleep
- client goal construction, timeout and cancel handling

## Why these blanks?
The blanks are chosen to target common LLM failure modes:
- forgetting `is_preempt_requested` and `set_preempted`
- mismatching field names vs .action
- incorrect SimpleActionServer/SimpleActionClient usage
- incorrect wait_for_result/cancel behavior

## Expected Outcome
- `catkin_make` succeeds
- Server and client can be completed to run together: client sends goal, server publishes feedback and returns result
- Manual verification: feedback prints and final success logged
