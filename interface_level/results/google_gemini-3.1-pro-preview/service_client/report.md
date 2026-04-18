# Benchmark Report: SERVICE_CLIENT

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| service_client/task_001_simple_service | FAILED | 7/8 | test_client_uses_rclpy_and_node | 4446 |
| service_client/task_002_custom_srv | FAILED | 6/7 | test_ros2_server_uses_node_subclass | 6155 |
| service_client/task_003_mp3_db_service | FAILED | 2/7 | test_ros2_client_uses_node, test_ros2_client_creates_service_client, test_client_populates_request_fields, test_client_accesses_response_list, test_client_uses_response_to_drive_loop | 5234 |
| service_client/task_005_gazebo | FAILED | 4/7 | test_return_depends_on_service_response, test_response_used_in_return, test_multiple_service_calls_use_response | 7371 |
| service_client/task_006_gazebo_set_get_state | TRANSLATION_FAILED | 0/0 | None | 0 |
| service_client/task_007_navigation_make_plan | FAILED | 0/4 | test_01_ros2_service_client_used, test_02_wait_for_service_retry, test_03_requestmap_has_mutex_lock_guard, test_04_response_map_flow | 83998 |
| service_client/task_008_clear_costmaps | FAILED | 4/6 | test_equivalence_layer_names_semantics_obstacles_static_map, test_equivalence_gates_both_obstacles_and_static_map | 8051 |
| service_client/task_009_moveit_planning_scene | FAILED | 3/4 | test_category_3_minimal_diff_hygiene_reset_and_not_keep_in_world | 8086 |
| service_client/task_010_controller_manager | FAILED | 2/5 | test_start_stop_controllers_semantics, test_list_controllers_semantics, test_reload_libraries_semantics | 7961 |
| service_client/task_011_robot_services | FAILED | 5/6 | test_target_camera_driver_is_referenced | 5423 |
