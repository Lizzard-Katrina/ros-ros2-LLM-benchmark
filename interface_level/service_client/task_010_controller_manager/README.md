# Task 010 — Controller Manager Services

# Task 010 — Controller Manager Service Client Migration

## 1. Brief Description

This task evaluates semantic preservation in migrating a ROS1 controller manager service client to ROS2 (rclpy).

The original ROS1 script coordinates multiple controller_manager services to:

- inspect controller states  
- start and stop controllers  
- reload controller libraries  
- optionally restore the controller configuration after reload  

The migration target is a ROS2 Python client that preserves the same behavioral semantics using ROS2 service clients.

This is an interface-level migration task. No server-side logic is involved.

---
source code:
`https://github.com/ros-controls/ros_control/blob/noetic-devel/controller_manager/src/controller_manager/controller_manager_interface.py`


## 2. Code Blanking Strategy

The original ROS1 script contains several helper functions that wrap controller_manager services.

To construct this benchmark task:

- High-level orchestration logic was preserved  
- Three semantically meaningful functions were blanked out:
  - `list_controllers`
  - `start_stop_controllers`
  - `reload_libraries`

These functions were chosen because they:

- Directly encode controller management semantics  
- Involve request/response reasoning  
- Represent common ROS1→ROS2 migration patterns  
- Cannot be solved by simple syntax rewriting  

Blanking focuses on *behavioral cores* rather than boilerplate, ensuring the task measures semantic reconstruction instead of API memorization.

---

## 3. Test Case Semantics

The oracle evaluates whether key behavioral semantics from ROS1 are preserved in the ROS2 implementation.  
It does NOT enforce coding style or formatting.

---

### Test Group A — ROS2 Client Usage

**What it checks**

- rclpy is used instead of rospy  
- ROS2 service clients are created  
- Asynchronous service calls are made  
- The client waits for service completion  

**Expected semantic outcome**

The implementation must follow ROS2 service-calling patterns and not fall back to ROS1 APIs.

---

### Test Group B — start_stop_controllers Semantics

**What it checks**

- SwitchController service type is used  
- Start/stop lists are forwarded into the request  
- The function’s return value depends on the service response  

**Expected semantic outcome**

The function must act as a true wrapper around the switch_controller service, preserving ROS1 behavior where success is determined by the response.

This detects implementations that:

- Ignore response values  
- Return constants  
- Omit stop/start propagation  

---

### Test Group C — list_controllers Semantics

**What it checks**

- ListControllers service is called  
- The response’s controller list is iterated  
- Claimed resource information is processed  
- Some human-readable output is produced  

**Expected semantic outcome**

The function should reconstruct the inspection behavior of the ROS1 script, where controller states and claimed interfaces are surfaced to the user.

This detects implementations that:

- Call the service but ignore results  
- Skip resource interpretation  
- Reduce the function to a stub  

---

### Test Group D — reload_libraries Semantics

**What it checks**

- ReloadControllerLibraries service is used  
- `force_kill` is propagated  
- A restore branch exists  
- Restore logic involves:
  - ListControllers  
  - LoadController  
  - SwitchController  
- Return value depends on service response  

**Expected semantic outcome**

When `restore=True`, the implementation must preserve the original ROS1 semantics:

1) snapshot controllers  
2) reload libraries  
3) reload prior controllers  
4) restart previously running ones  

This detects partial migrations where reload is performed but restoration semantics are lost.

---

## Summary

This task measures whether a system can preserve **controller management semantics** across ROS versions, not whether it can mechanically translate APIs.

Correct solutions demonstrate:

- response-aware logic  
- multi-service orchestration  
- preservation of restore semantics  

rather than superficial syntactic similarity.
