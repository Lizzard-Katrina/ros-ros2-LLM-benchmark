# Task 012 — Move Arm Cartesian Action

## Purpose
Benchmark ROS1 → ROS2 translation of a MoveIt Cartesian action server. Server receives Cartesian goal (x, y, z + orientation), computes IK, executes trajectory, and publishes continuous end-effector feedback.

## TODO Versions

| File | TODO Description | Reason for TODO | Expected Outcome |
|------|----------------|----------------|----------------|
| move_arm_cartesian_action_server_todo1.py | create SimpleActionServer | Critical server creation | Server object exists and ready to start |
| move_arm_cartesian_action_server_todo2.py | start the server | Accept goals | Server is running and can accept goals |
| move_arm_cartesian_action_server_todo3.py | parse Cartesian goal | Parse x,y,z,orientation | Goal correctly interpreted for IK |
| move_arm_cartesian_action_server_todo4.py | execute motion | Compute IK, execute trajectory, publish feedback | Continuous feedback; trajectory executed; result success |
| utils/ik_solver.py | compute IK | Encapsulate inverse kinematics | Correct joint angles returned (stub/dummy allowed) |
| utils/trajectory_builder.py | build trajectory | Encapsulate trajectory building | Smooth joint trajectory returned (stub/dummy allowed) |
