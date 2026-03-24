# Benchmark Report: SERVICE_CLIENT

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| service_client/task_001_simple_service | SUCCESS | 8/8 | None | 945 |
| service_client/task_002_custom_srv | FAILED | 6/7 | test_server_defines_handler_and_accesses_request | 919 |
| service_client/task_003_mp3_db_service | FAILED | 4/7 | test_client_populates_request_fields, test_client_accesses_response_list, test_client_uses_response_to_drive_loop | 1495 |
| service_client/task_005_gazebo | FAILED | 2/7 | test_return_depends_on_service_response, test_failure_path_exists, test_response_used_in_return, test_exception_handled, test_multiple_service_calls_use_response | 1841 |
| service_client/task_006_gazebo_set_get_state | FAILED | 6/8 | test_03_retry_wait_for_service_with_shutdown_guard, test_05_full_service_call_chain_success_and_failure_handling | 1869 |
| service_client/task_007_navigation_make_plan | FAILED | 0/4 | test_01_ros2_service_client_used, test_02_wait_for_service_retry, test_03_requestmap_has_mutex_lock_guard, test_04_response_map_flow | 22300 |
| service_client/task_008_clear_costmaps | FAILED | 3/6 | test_equivalence_reset_distance_is_expressed, test_equivalence_layer_names_semantics_obstacles_static_map, test_equivalence_gates_both_obstacles_and_static_map | 2262 |
| service_client/task_009_moveit_planning_scene | SUCCESS | 4/4 | None | 5179 |
| service_client/task_010_controller_manager | FAILED | 3/5 | test_start_stop_controllers_semantics, test_reload_libraries_semantics | 2288 |
| service_client/task_011_robot_services | FAILED | 2/6 | test_target_camera_driver_is_referenced, test_init_sets_up_configuration_interface, test_disable_auto_sets_both_flags_false, test_enable_auto_sets_both_flags_true | 1130 |
