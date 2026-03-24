{
  "task_name": "task_005_gazebo_service_clients",
  "description": "Implement ROS2 service client functions for interacting with Gazebo: spawn_sdf_model_client, spawn_urdf_model_client, set_model_configuration_client. Focus on handling service calls correctly, using the service response to determine success/failure, catching exceptions, and avoiding hardcoded return values.",
  "source_files": [
    "gazebo_interface.py"
  ],
  "skill_focus": [
    "ROS2 service client creation and usage",
    "Response handling and exception mapping",
    "Writing robust service client logic"
  ],
  "oracle_test_coverage": [
    "Return values depend on service response",
    "Failure paths return False",
    "Responses are used in return logic",
    "No trivial hardcoded True/False assignments",
    "Exceptions are properly handled",
    "Multiple service calls correctly use their responses",
    "Syntax is valid Python"
  ],
  "difficulty": "Medium"
}
