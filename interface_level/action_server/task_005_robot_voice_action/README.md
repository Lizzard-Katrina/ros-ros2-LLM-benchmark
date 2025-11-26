# Task 005 — Robot Voice Command Action

## Purpose
Benchmark ROS1 → ROS2 translation of an action server. The server receives voice commands, executes the command, publishes feedback, and returns success/failure.

## TODO Versions

| File | TODO Description | Reason for TODO | Expected Outcome |
|------|----------------|----------------|----------------|
| voice_action_server_todo1.py | create SimpleActionServer | Critical: server must be created | Server object exists and ready to start |
| voice_action_server_todo2.py | start the server | Server start needed to accept goals | Server is running and can accept goals |
| voice_action_server_todo3.py | parse command | Test understanding of goal parsing | Command string is correctly extracted from goal |
| voice_action_server_todo4.py | publish feedback | Key for client monitoring | Feedback messages sent during execution |
| voice_action_server_todo5.py | set result | Validate result handling | Result.success reflects action outcome |
| utils/goal_builder.py | attach command to goal | Encapsulates goal creation | Goal contains correct command string |
| utils/feedback_handler.py | print feedback | Validate feedback handling | Feedback progress displayed in console/logs |
