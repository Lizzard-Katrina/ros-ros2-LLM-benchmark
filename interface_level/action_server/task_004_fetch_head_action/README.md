# Task 004 — Fetch Head Pointing Action

## Purpose
Benchmark ROS1 → ROS2 translation of an action server. The server receives a target TF, moves the robot head, publishes feedback, and returns success/failure.

## TODO Versions

| File | TODO Description | Reason for TODO | Expected Outcome |
|------|----------------|----------------|----------------|
| head_action_server_todo1.py | create SimpleActionServer | Critical: server must be created | Server object exists and ready to start |
| head_action_server_todo2.py | start the server | Server start needed to accept goals | Server is running and can accept goals |
| head_action_server_todo3.py | parse target TF | Test understanding of goal parsing | Target frame string is correctly extracted |
| head_action_server_todo4.py | publish feedback | Key for client monitoring | Feedback messages sent during execution |
| head_action_server_todo5.py | set result | Validate result handling | Result.success reflects action outcome |
| utils/goal_builder.py | attach target frame to goal | Encapsulates goal creation | Goal contains correct target_frame |
| utils/feedback_handler.py | print feedback progress | Validate feedback handling | Feedback progress displayed in console/logs |
