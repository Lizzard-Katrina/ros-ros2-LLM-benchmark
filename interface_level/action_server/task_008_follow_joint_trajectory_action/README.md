# Task 008 — Follow Joint Trajectory Action

## Purpose
Benchmark ROS1 → ROS2 translation of an action server. The server executes FollowJointTrajectory goals, provides feedback on joint positions, and reports success/failure.

## TODO Versions

| File | TODO Description | Reason for TODO | Expected Outcome |
|------|----------------|----------------|----------------|
| joint_trajectory_action_server_todo1.py | create SimpleActionServer | Critical: server must be created | Server object exists and ready to start |
| joint_trajectory_action_server_todo2.py | start the server | Server start needed to accept goals | Server is running and can accept goals |
| joint_trajectory_action_server_todo3.py | parse goal | Test goal parsing | Goal points and joint names correctly extracted |
| joint_trajectory_action_server_todo4.py | execute trajectory | Key trajectory execution logic | Robot joint positions updated; feedback published |
| joint_trajectory_action_server_todo5.py | set result | Validate result handling | Result.error_code reflects execution outcome |
| utils/goal_builder.py | attach joints and points to goal | Encapsulates goal creation | Goal contains correct joint_names and trajectory points |
| utils/feedback_handler.py | print feedback | Validate feedback handling | Feedback actual.positions displayed in console/logs |
