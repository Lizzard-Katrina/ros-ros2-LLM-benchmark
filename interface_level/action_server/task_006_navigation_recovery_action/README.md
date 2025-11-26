# Task 006 — Navigation Recovery Action

## Purpose
Benchmark ROS1 → ROS2 translation of an action server. The server executes recovery actions when a goal fails (e.g., spin, clear costmap) and reports success/failure.

## TODO Versions

| File | TODO Description | Reason for TODO | Expected Outcome |
|------|----------------|----------------|----------------|
| recovery_action_server_todo1.py | create SimpleActionServer | Critical: server must be created | Server object exists and ready to start |
| recovery_action_server_todo2.py | start the server | Server start needed to accept goals | Server is running and can accept goals |
| recovery_action_server_todo3.py | parse goal | Test understanding of goal parsing | Recovery type correctly extracted from goal |
| recovery_action_server_todo4.py | execute recovery | Key recovery logic | Recovery actions executed (spin / clear costmap) |
| recovery_action_server_todo5.py | set result | Validate result handling | Result.success reflects recovery outcome |
| utils/goal_builder.py | attach recovery type to goal | Encapsulates goal creation | Goal contains correct recovery_type |
| utils/feedback_handler.py | print feedback | Validate feedback handling | Feedback progress displayed in console/logs |
