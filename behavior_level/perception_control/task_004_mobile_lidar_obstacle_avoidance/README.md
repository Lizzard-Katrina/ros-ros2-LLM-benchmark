# Task 004 — Mobile LiDAR Obstacle Avoidance (ROS1 → ROS2)

## 1) Brief Description

This task benchmarks an LLM’s ability to translate a ROS1 (rospy) reactive LiDAR obstacle avoidance script into a ROS2 (rclpy) node **while preserving the original algorithmic structure**.

The reference behavior is a **closed loop**:

`LaserScan (/scan) → 12-sector partition → obstacle lists per sector → decision (act/angular_vel/sleep) → Twist (/cmd_vel) → repeat at ~10Hz`

This benchmark is **fidelity-oriented**: solutions that redesign the algorithm (even if “reasonable”) may fail by design.

---
source code:
```https://github.com/Rad-hi/Obstacle-Avoidance-ROS/blob/main/scripts/old_approach/laser_obstacle_avoid_360_node.py```


## 2) Hollowed (TODO) Parts and Why

We hollowed out the **core closed-loop logic** rather than boilerplate, so the model must reconstruct behavior from surrounding context and constants.

### Hollowed parts
- **Perception / partitioning**: region extraction from `LaserScan.ranges`
- **Decision logic**: choosing a heading using region “clearness” + signed deviation cost
- **Control output**: generating a `Twist` consistent with the decision state
- **ROS2 structure**: correct rclpy node wiring, timers, publishers/subscribers, lifecycle

### Why these were chosen
- They capture what most strongly differentiates a *faithful translation* from a template ROS2 node:
  - 12-sector partitioning by slicing
  - per-region obstacle filtering with threshold + infinity handling
  - signed cost table (`Regions_Distances`) and use of absolute deviation cost
  - normalized angular velocity derived from signed regional distance
  - geometric “turn duration” computation (`sleep`) retained as state
  - two motion modes (TRANS avoidance vs NORMAL cruise)
- These behaviors are **statically checkable** via pattern matching (regex/string), enabling fast oracle tests (<1s) without execution.

---

## 3) Oracle Tests: Rationale + Expected Outcome to Pass

The oracle uses pytest tests that validate source code using **pattern matching only** (no runtime, no compilation).  
Each test checks one semantic concept. Passing requires the expected patterns to be present.

### Test 1 — ROS2 scaffold + no ROS1 remnants
**Why:** Prevent partial ports that still use ROS1 APIs.
**Expected to pass:**
- Uses `rclpy` and `Node` (`from rclpy.node import Node`)
- Does **not** use `rospy`

---

### Test 2 — ROS2 lifecycle + node creation
**Why:** Enforces standard ROS2 node lifecycle.
**Expected to pass:**
- Calls `rclpy.init(...)`, `rclpy.spin(...)`, `rclpy.shutdown(...)`
- Creates a node (Node subclass or `Node("name")` / `rclpy.create_node("name")`)

---

### Test 3 — Correct ROS2 interfaces (/scan subscription + /cmd_vel publisher)
**Why:** Ensures the translation preserves I/O contracts.
**Expected to pass:**
- Subscribes to `/scan` using `LaserScan`
- Publishes to `/cmd_vel` using `Twist`
- Imports `LaserScan` and `Twist` from ROS2 message modules

---

### Test 4 — 10Hz loop implemented as ROS2 timer + publishes Twist
**Why:** ROS1 uses `Rate(10)`; ROS2 translation should use a timer instead of a busy loop.
**Expected to pass:**
- Uses `create_timer(0.1, ...)` (or an equivalent 10Hz period)
- Calls `.publish(...)` on a publisher

---

### Test 5 — Keeps reference 12-sector structure and thresholds
**Why:** The benchmark targets structure-preserving translation of the original 12-sector algorithm.
**Expected to pass:**
- Defines `REGIONS` with the 12 named regions:
  `front_C, front_L, left_R, left_C, left_L, back_R, back_C, back_L, right_R, right_C, right_L, front_R`
- Retains `OBSTACLE_DIST`, `REGIONAL_ANGLE`, and `PI/pi` concepts

---

### Test 6 — **Hard fidelity gate**: partition ranges via slicing into contiguous chunks
**Why:** The ROS1 reference partitions `ranges` into regions using slicing; this is a key structural signature.
**Expected to pass:**
- Reads `msg.ranges` / `scan.ranges`
- Uses slicing like `ranges[a:b]` to form regions
- Iterates regions with `enumerate(REGIONS)` (or equivalent)

> Note: Even logically equivalent implementations that avoid slicing may fail by design.

---

### Test 7 — IdentifyRegions strategy fidelity: threshold filtering + infinity handling + per-region lists
**Why:** Preserves the perception semantics of “obstacle points in each sector.”
**Expected to pass:**
- Compares readings to `OBSTACLE_DIST`
- Handles infinity values (`inf`, `float('inf')`, or `isfinite`)
- Stores obstacle lists in a `Regions_Report` dict

---

### Test 8 — Decision uses signed cost table + absolute deviation cost
**Why:** The reference uses a signed “deviation cost” map around the goal region and uses `abs(...)` for cost.
**Expected to pass:**
- Defines `Regions_Distances` as a dict
- Contains **both negative and positive** integer costs
- Uses `abs(...)` in the decision logic

---

### Test 9 — Control law fidelity: act + normalized angular velocity + turn-duration state + motion modes
**Why:** This captures the reference decision-to-control linkage.
**Expected to pass:**
- Maintains `Urgency_Report` with keys `act`, `angular_vel`, `sleep`
- `angular_vel` depends on a signed regional distance with normalization/sign logic
- Keeps `sleep`/turn-duration computation tied to `REGIONAL_ANGLE`, `180`, `PI`, `TRANS_ANG_VEL`
- Implements two motion modes:
  - avoidance uses `TRANS_LIN_VEL` and `angular.z = Urgency_Report["angular_vel"]`
  - normal uses `NORMAL_LIN_VEL` and zero angular velocity
- Branches on `Urgency_Report["act"]` to switch between modes

---

## Notes

- Tests are **static** and must run in <1 second.
- This benchmark measures **translation fidelity** to the reference algorithm, not general obstacle avoidance quality.
- Architecturally valid redesigns may fail (by design) if they do not preserve the 12-sector slicing structure and signed-cost decision logic.
