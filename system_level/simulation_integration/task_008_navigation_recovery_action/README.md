# Task 008 — Navigation Recovery Action

## Purpose
Benchmark ROS1 → ROS2 translation of an action server. The server executes recovery actions when a goal fails (e.g., spin, clear costmap) and reports success/failure.

## Source code:

Use the entire code file ```https://gist.github.com/histvan95/261482184e36bb238d9c45a361586316```

## Test Categories

### 1. ROS1 Artifacts Removal
- **Purpose:** Ensure no ROS1 APIs or headers remain.
- **Expected Outcome:** Code should compile/run without ROS1 dependencies.

### 2. Subscriber Creation
- **Purpose:** Verify that the node subscribes to `/gazebo/model_states` and `/gazebo/link_states`.
- **Expected Outcome:** Both subscriptions must exist to receive model and link states.

### 3. Service Client Usage
- **Purpose:** Check that the node creates a service client for `/gazebo/set_model_state` and calls `set_model_state` correctly.
- **Expected Outcome:** Service client exists and is called with proper Pose and Twist arguments.

### 4. Pose and Twist Initialization
- **Purpose:** Ensure geometry_msgs::Pose and geometry_msgs::Twist objects are constructed and initialized.
- **Expected Outcome:** Pose and Twist objects are present and passed into the service call.

## How to Pass

1. Remove all ROS1 includes and objects.
2. Translate subscriptions and service client to ROS2 semantics.
3. Ensure the logic block updates the model state successfully via service call.
4. Pose and Twist messages must be initialized before calling the service.

## Notes

- This task focuses on **simulation integration**, not real robot control.
- The logic to update model state is considered a single closed-loop for verification.
- Tests check **code structure** rather than runtime Gazebo simulation.

## TODO Versions
