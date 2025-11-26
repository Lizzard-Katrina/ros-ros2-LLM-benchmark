# Task 007 — Move Base Navigation Action

## Purpose
Benchmark ROS1 → ROS2 translation of an action server. The server executes navigation commands sent by clients, provides feedback on position, and reports success/failure.

## TODO Versions

| File | TODO Description | Reason for TODO | Expected Outcome |
|------|----------------|----------------|----------------|
| move_base_action_server_todo1.py | create SimpleActionServer | Critical: server must be created | Server object exists and ready to start |
| move_base_action_server_todo2.py | start the server | Server start needed to accept goals | Server is running and can accept goals |
| move_base_action_server_todo3.py | parse goal | Test goal parsing | Goal x, y, orientation correctly extracted |
| move_base_action_server_todo4.py | execute navigation | Key navigation logic | Robot moves toward goal with feedback |
| move_base_action_server_todo5.py | set result | Validate result handling | Result.success reflects navigation outcome |
| utils/goal_builder.py | attach x, y, orientation to goal | Encapsulates goal creation | Goal contains correct target pose |
| utils/feedback_handler.py | print feedback | Validate feedback handling | Feedback base_position displayed in console/logs |
