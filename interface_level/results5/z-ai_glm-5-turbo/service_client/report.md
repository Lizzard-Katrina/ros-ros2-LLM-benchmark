# System Level Benchmark Report: SERVICE_CLIENT

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| service_client/task_001_simple_service | SUCCESS | 8/8 | None | 863 |
| service_client/task_002_custom_srv | FAILED | 6/7 | test_ros2_server_uses_node_subclass | 887 |
| service_client/task_003_mp3_db_service | FAILED | 2/7 | test_ros2_client_uses_node, test_ros2_client_creates_service_client, test_client_populates_request_fields, test_client_accesses_response_list, test_client_uses_response_to_drive_loop | 1372 |
| service_client/task_005_gazebo | FAILED | 5/7 | test_response_used_in_return, test_multiple_service_calls_use_response | 1797 |
| service_client/task_006_gazebo_set_get_state | FAILED | 6/8 | test_03_retry_wait_for_service_with_shutdown_guard, test_05_full_service_call_chain_success_and_failure_handling | 1637 |
| service_client/task_007_navigation_make_plan | FAILED | 3/4 | test_03_requestmap_has_mutex_lock_guard | 30866 |
| service_client/task_008_clear_costmaps | FAILED | 3/6 | test_equivalence_reset_distance_is_expressed, test_equivalence_layer_names_semantics_obstacles_static_map, test_equivalence_gates_both_obstacles_and_static_map | 2117 |
| service_client/task_009_moveit_planning_scene | SUCCESS | 4/4 | None | 3981 |
| service_client/task_010_controller_manager | FAILED | 4/5 | test_reload_libraries_semantics | 2030 |
| service_client/task_011_robot_services | FAILED | 3/6 | test_target_camera_driver_is_referenced, test_disable_auto_sets_both_flags_false, test_enable_auto_sets_both_flags_true | 1047 |
