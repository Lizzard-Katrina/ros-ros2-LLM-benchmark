# Task 010 — Drone Takeoff-Land Action

## Purpose
Benchmark ROS1 → ROS2 translation of an action server. The server executes drone takeoff/land goals, provides continuous altitude feedback, and reports success.

## TODO Versions

| File | TODO Description | Reason for TODO | Expected Outcome |
|------|----------------|----------------|----------------|
| drone_takeoff_action_server_todo1.py | create SimpleActionServer | Critical: server must be created | Server object exists and ready to start |
| drone_takeoff_action_server_todo2.py | start the server | Server start needed to accept goals | Server is running and can accept goals |
| drone_takeoff_action_server_todo3.py | parse goal | Test goal parsing | Goal correctly identifies takeoff/land and altitude |
| drone_takeoff_action_server_todo4.py | execute motion | Key motion execution | Drone altitude updated; feedback published |
| utils/goal_builder.py | attach takeoff/land and altitude to goal | Encapsulates goal creation | Goal contains correct takeoff/altitude |
| utils/feedback_handler.py | print feedback | Validate feedback handling | Feedback altitude displayed in console/logs |
