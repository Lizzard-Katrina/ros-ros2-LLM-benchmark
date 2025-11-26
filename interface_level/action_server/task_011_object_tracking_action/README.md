# Task 011 — Object Tracking Action

## Purpose
Benchmark ROS1 → ROS2 translation of an action server. The server receives object tracking goals, subscribes to sensor data, runs (simplified) perception pipeline, publishes pose feedback, and returns success.

## TODO Versions

| File | TODO Description | Reason for TODO | Expected Outcome |
|------|----------------|----------------|----------------|
| object_tracking_action_server_todo1.py | create SimpleActionServer | Critical server creation | Server object exists and ready to start |
| object_tracking_action_server_todo2.py | start the server | Accept goals | Server is running and can accept goals |
| object_tracking_action_server_todo3.py | parse goal | Parse object id and detection params | Goal correctly identified for tracking |
| object_tracking_action_server_todo4.py | execute tracking | Execute simplified perception pipeline | Feedback published; object pose updated; result success |
| utils/goal_builder.py | build goal object | Encapsulate goal creation | Goal object correctly populated |
| utils/feedback_handler.py | print feedback | Validate feedback handling | Feedback pose displayed in console/logs |
