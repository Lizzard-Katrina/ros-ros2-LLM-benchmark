# Task 007 — Navigation Make Plan (Service Client)

## 1. Brief Description

This task evaluates whether an LLM can correctly migrate a **ROS1 service-client workflow to ROS2** while preserving not only API usage but also key **semantic behaviors**.

The focus is on the `requestMap()` function in AMCL, which requests a static map from a service and feeds it into the localization pipeline.

Unlike simple API translation tasks, this task also checks whether the model preserves **implicit system semantics** such as:

- Service request workflow  
- Proper response handling  
- Data flow into downstream logic  
- Thread-safety intent from the original ROS1 code  

The goal is to test **interface-level migration with semantic awareness**, not just syntax translation.

---
source code: 
`https://github.com/ros-planning/navigation/blob/noetic-devel/amcl/src/amcl_node.cpp`

## 2. Why This Part of the Source Code Was Masked

We mask (remove) the implementation of:

`AmclNode::requestMap()`

### Reasoning

In ROS1, this function:

- Calls a map service (`static_map`)
- Retries until success
- Locks a configuration mutex
- Passes the returned map to `handleMapMessage`

This makes it ideal because it contains:

✅ A clear service-client pattern  
✅ Retry logic  
✅ Shared-state access  
✅ Downstream data flow  
✅ Hidden semantic constraints (mutex use)

It is small enough for benchmarking but rich enough to reveal whether the model understands:

> “How this function fits into the AMCL pipeline.”

---

## 3. Oracle Testcases

Only **4 high-value tests** are used.  
Each checks one independent semantic concept.

---

### ✅ Test 1 — ROS2 Service Client Creation

**What it checks**

The model must use a ROS2 service client with the correct service type.

**Design Idea**

We verify presence of:

`create_client<nav_msgs::srv::GetMap>`

This ensures:

- ROS2 API usage
- Correct service type migration

**Expected Outcome to Pass**

The translated code creates a ROS2 client for `nav_msgs::srv::GetMap`.

---

### ✅ Test 2 — Waiting for Service Availability

**What it checks**

The model must wait for the service to become available before sending a request.

**Design Idea**

We search for:

`wait_for_service(...)`

This captures the retry/wait semantics of the original ROS1 code.

**Expected Outcome to Pass**

The code includes a service-availability wait before request sending.

---

### ✅ Test 3 — Mutex Lock Semantics (Key Test)

**What it checks**

Whether the model preserves the original ROS1 thread-safety intent.

ROS1 code used:

`scoped_lock(configuration_mutex_)`

This indicates that requesting and consuming the map touches shared state.

**Design Idea**

We check for:

- `scoped_lock`, `lock_guard`, or `unique_lock`
- AND reference to `configuration_mutex_`

This is a **semantic proxy** for thread-safety awareness.

**Expected Outcome to Pass**

The code locks `configuration_mutex_` (or equivalent) when requesting/processing the map.

This is the most discriminative test.

---

### ✅ Test 4 — Response Map Data Flow

**What it checks**

The service response must actually be used and fed into the pipeline.

Specifically:

`handleMapMessage(response->map)`

