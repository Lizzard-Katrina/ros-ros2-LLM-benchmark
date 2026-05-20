# System Level Benchmark Report: SERVICE_CLIENT

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| service_client/task_001_simple_service | FAILED | 7/8 | test_client_uses_rclpy_and_node | 893 |
| service_client/task_002_custom_srv | FAILED | 6/7 | test_ros2_server_uses_node_subclass | 1073 |
| service_client/task_003_mp3_db_service | FAILED | 2/7 | test_ros2_client_uses_node, test_ros2_client_creates_service_client, test_client_populates_request_fields, test_client_accesses_response_list, test_client_uses_response_to_drive_loop | 1935 |
| service_client/task_005_gazebo | FAILED | 4/7 | test_return_depends_on_service_response, test_response_used_in_return, test_multiple_service_calls_use_response | 1801 |
| service_client/task_006_gazebo_set_get_state | FAILED | 7/8 | test_05_full_service_call_chain_success_and_failure_handling | 1928 |
| service_client/task_007_navigation_make_plan | FAILED | 3/4 | test_03_requestmap_has_mutex_lock_guard | 28305 |
| service_client/task_008_clear_costmaps | SUCCESS | 6/6 | None | 2313 |
| service_client/task_009_moveit_planning_scene | FAILED | 3/4 | test_category_2_core_transition_attach_step_world_remove_and_robot_attach | 4043 |
| service_client/task_010_controller_manager | FAILED | 2/5 | test_start_stop_controllers_semantics, test_list_controllers_semantics, test_reload_libraries_semantics | 2036 |
| service_client/task_011_robot_services | FAILED | 5/6 | test_target_camera_driver_is_referenced | 1442 |
