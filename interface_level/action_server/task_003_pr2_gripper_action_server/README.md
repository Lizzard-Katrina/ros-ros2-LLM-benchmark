# Task 003 — PR2 Gripper Action Server

## Purpose
Test ROS1 → ROS2 translation of an action server. The action server handles grasp commands, publishes feedback, and returns results.

## TODO Versions

| File | TODO Description | Reason for TODO | Expected Outcome |
|------|----------------|----------------|----------------|
| gripper_action_server_todo1.py | create SimpleActionServer | Critical step: server must be created correctly | Server object is created and ready to start |
| gripper_action_server_todo2.py | start the server | Server start is required before accepting goals | Server starts and can accept action goals from clients |
| gripper_action_server_todo3.py | publish feedback | Feedback is key for client monitoring; tests LLM understanding of execute_cb | Feedback messages are sent to client during goal execution |
| gripper_action_client.py | wait for server / process feedback | Client must wait and handle feedback properly | Client waits until server is available and prints feedback updates |
| utils/goal_builder.py | attach command to goal | Encapsulates goal creation logic | Goal object contains the correct position and effort commands |
| utils/feedback_handler.py | print feedback | Validate feedback handling logic | Feedback values are displayed in console/logs |

