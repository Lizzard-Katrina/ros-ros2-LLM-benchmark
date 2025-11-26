# Task 009 — MoveIt Motion Planning Action

## Purpose
Benchmark ROS1 → ROS2 translation of an action server. The server executes motion planning goals using MoveIt, provides feedback on motion execution, and reports success/failure.

## TODO Versions

| File | TODO Description | Reason for TODO | Expected Outcome |
|------|----------------|----------------|----------------|
| moveit_action_server_todo1.py | create SimpleActionServer | Critical: server must be created | Server object exists and ready to start |
| moveit_action_server_todo2.py | start the server | Server start needed to accept goals | Server is running and can accept goals |
| moveit_action_server_todo3.py | parse goal | Test goal parsing | Goal target pose correctly extracted |
| moveit_action_server_todo4.py | execute motion | Key motion planning execution | Robot motion executed; feedback published |
| moveit_action_server_todo5.py | set result | Validate result handling | Result.error_code reflects motion planning outcome |
| utils/goal_builder.py | attach target_pose to goal | Encapsulates goal creation | Goal contains correct target pose |
| utils/feedback_handler.py | print feedback | Validate feedback handling | Feedback state displayed in console/logs |
