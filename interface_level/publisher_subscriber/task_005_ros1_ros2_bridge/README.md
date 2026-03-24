# Task 005 — ROS1 to ROS2 Bridge Communication

## Goal
This task evaluates the model's ability to translate ROS1 publisher logic and ROS2 subscriber logic in a cross-system communication scenario using `ros1_bridge`.

## Description
A ROS1 publisher publishes to `/chatter`.  
The ROS1–ROS2 bridge relays the message to ROS2.  
A ROS2 listener node should receive the message.

Source reference:
```https://github.com/ros2/ros1_bridge```

## Directory Structure
(standard as previous tasks)

## Docker
- ROS2 workspace

## Oracle tests

### Test Items
1. **Module Importable:**  
   The translated `talker.py` must exist and be importable.

2. **Main Function:**  
   The module defines a `main()` function.

3. **Message Construction:**  
   A `String` message can be instantiated and its `data` field can be assigned.

4. **Correct ROS2 API Usage:**  
   The translated code should use `rclpy` and should not contain any leftover `rospy` calls.

5. **Publisher Definition:**  
   The module defines a publisher for the `/chatter` topic.

### Expected Outcome (if fully correct)
- The module imports without errors.  
- `main()` function exists.  
- `String` message can be created and `data` assigned correctly.  
- No `rospy` calls remain in the translated ROS2 code.  
- Publisher is properly defined for `/chatter`.

