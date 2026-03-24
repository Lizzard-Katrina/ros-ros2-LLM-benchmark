# Task 003 — ROS1 → ROS2 Translation Benchmark (Kobuki Joystick)

## 1) Brief Description

This task evaluates whether an LLM can **faithfully translate** a ROS1 C++ joystick teleoperation node into **ROS2 C++ (rclcpp)**, **without running or compiling code**.  
The oracle is implemented as **pytest regex/string pattern tests** that check for key semantic behaviors and API migrations.

**Source behavior summary (ROS1 reference):**
- Read Linux joystick events from a device file (e.g., `/dev/input/js0`) using `open()` and `read()` with `js_event`.
- Use DS4 mappings:
  - L1 button enables/disables robot motor power.
  - L3_Y controls `Twist.linear.x`.
  - R3_X controls `Twist.angular.z`.
- Normalize axis values by `32767.0` and apply `scale_linear` / `scale_angular`.
- Only publish `cmd_vel` when enabled and when command is non-zero.
- On disable, publish a stop Twist and send motor power OFF.

---
reference code:
```https://github.com/slamcore/slamcore-ros1-examples/blob/master/src/slamcore_ros1_examples/src/kobuki_joystick.cpp```

---


## 2) Why This Code Was “Holed Out” (挖空理由)

We selected this file because it combines **multiple migration-relevant concepts** that commonly break during ROS1→ROS2 translation:

- **ROS framework migration:** `roscpp` APIs (`ros::init`, `ros::NodeHandle`, `advertise`, `ros::ok`) must be replaced with ROS2 (`rclcpp`, publishers, parameters, spin lifecycle).
- **OS-level IO fidelity:** The logic depends on Linux joystick device events (`js_event`), non-blocking reads, and filtering initialization events. This prevents a “template ROS2 node” from passing.
- **Behavioral gating & safety:** The node must implement enable/disable gating and safety stop behavior, which are subtle and easy to omit.
- **Deterministic numeric transformation:** The axis normalization (`/32767.0`) and scaling must remain consistent with the reference.
- **Benchmark robustness:** These semantics can be validated via **static pattern matching** without requiring compilation or runtime.

The “holes” are intended to force the translator to reconstruct **core behavioral structure** rather than superficially rename APIs.

---

## 3) Oracle Tests (Design Rationale + Expected Outcome)

The oracle uses **5 independent tests**. Each test is designed to validate a high-level semantic concept tied to the ROS1 reference. To pass, the translated ROS2 code should include the **expected outcome** below.

> **Important:** Tests use regex/string matching only. Passing requires that the translated code includes recognizable patterns reflecting reference behavior.

---

### Test 1 — ROS2-only & Lifecycle Correctness  
**What it checks:**  
- Must use ROS2 (`rclcpp`) and ROS2 logging (`RCLCPP_*`).
- Must have a ROS2 lifecycle structure (`rclcpp::init`, some spin/executor, `rclcpp::shutdown`).
- Must not contain ROS1 `roscpp` surface (`ros/ros.h`, `ros::NodeHandle`, `ROS_*`).

**Why this test exists:**  
A frequent failure mode is “hybrid” code that still uses ROS1 APIs or does not properly spin/shutdown in ROS2.

**Expected outcome to pass:**  
The translated code includes:
- `#include "rclcpp/rclcpp.hpp"` (or equivalent)
- `rclcpp::init(...)`, `rclcpp::spin(...)` (or executor spin), and `rclcpp::shutdown()`
- `RCLCPP_INFO/...` logging
- No `ros::NodeHandle`, no `ros::init`, no `ROS_INFO_*`

---

### Test 2 — Linux Joystick IO Fidelity (Strict)  
**What it checks:**  
- Uses Linux joystick device reading (`open`, `read`, `js_event`).
- Opens in non-blocking mode (`O_NONBLOCK`).
- **Strict fidelity requirement:** clears init flag exactly like reference:
  - `event.type &= ~JS_EVENT_INIT;`

**Why this test exists:**  
The reference code depends on Linux joystick semantics; omitting `JS_EVENT_INIT` filtering can cause initialization events to be misinterpreted as real input. This is a subtle behavior many translations miss.

**Expected outcome to pass:**  
The translated code includes:
- `#include <fcntl.h>` and `#include <unistd.h>`
- `open(... O_RDONLY ... O_NONBLOCK ...)`
- `js_event event;` (or equivalent)
- `read(fd, &event, sizeof(event))`
- `event.type &= ~JS_EVENT_INIT;` (exact behavior)

---

### Test 3 — DS4 Mapping Structures & Usage  
**What it checks:**  
- Defines DS4 enums with reference names:
  - `enum DS4_BUTTONS { ... L1 ... }`
  - `enum DS4_AXIS { ... L3_Y ... R3_X ... }`
- Uses joystick event types:
  - `event.type == JS_EVENT_BUTTON`
  - `event.type == JS_EVENT_AXIS`
- Compares `event.number` to the DS4 enum members:
  - `event.number == DS4_BUTTONS::L1`
  - `event.number == DS4_AXIS::L3_Y`
  - `event.number == DS4_AXIS::R3_X`

**Why this test exists:**  
A common shortcut is replacing DS4 mappings with numeric constants or rewriting control flow. This benchmark is **fidelity-oriented**, so we require recognizable DS4 mapping structure.

**Expected outcome to pass:**  
The translated code must visibly preserve DS4 mapping symbols and use them in event routing logic.

---

### Test 4 — L1 Enable/Disable Semantics (Press vs Release)  
**What it checks:**  
- Explicit press/release handling:
  - enable when `event.value == 1`
  - disable when `event.value == 0`
- Enabled state toggling (e.g., `m_enabled = true/false`)
- Motor power semantics:
  - publish `MotorPower ON` on enable
  - publish `MotorPower OFF` on disable
- Safety stop on disable:
  - set `linear.x = 0` and `angular.z = 0`

**Why this test exists:**  
This is the core behavioral contract: the robot must only drive when the operator holds L1, and it must stop safely when released.

**Expected outcome to pass:**  
The translated code includes:
- `event.value == 1` → enable + set enabled flag true + publish MotorPower ON
- `event.value == 0` → disable + set enabled flag false + zero Twist + publish MotorPower OFF

---

### Test 5 — Axis Scaling & Publish Gating Fidelity  
**What it checks:**  
- Normalization uses the same constant: `32767` or `32767.0`.
- Twist computations match reference shape (must include):
  - a leading negative sign `-`
  - `event.value`
  - division by `32767`
  - multiplication by `scale_linear` / `scale_angular`
- Gated publishing must match reference form:
  - publish `cmd_vel` only when enabled AND `(linear.x != 0 || angular.z != 0)`

**Why this test exists:**  
Two common translation failures are:
1) changing the scaling semantics or sign conventions,
2) publishing continuously even when disabled or idle.

This test enforces fidelity to the numeric and gating logic.

**Expected outcome to pass:**  
The translated code includes patterns equivalent to:
- `twist.linear.x = -event.value / 32767.0 * scale_linear;`
- `twist.angular.z = -event.value / 32767.0 * scale_angular;`
- `if (enabled && (twist.linear.x != 0 || twist.angular.z != 0)) publish(cmd_vel);`

---

## Notes

- The oracle is **static** (pattern matching only) and is designed for runtime under 1 second.
- The benchmark intentionally prioritizes **translation fidelity** over alternative valid redesigns (e.g., subscribing to `/joy` messages instead of reading `/dev/input/js*`).
- If a translated solution chooses a different architecture, it may be valid ROS2 code but can still fail this benchmark by design.
