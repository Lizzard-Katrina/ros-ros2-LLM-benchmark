# Benchmark Report: SERVICE_CLIENT

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| service_client/task_001_simple_service | FAILED | 7/8 | test_client_calls_service_async | 1368 |
| service_client/task_002_custom_srv | FAILED | 6/7 | test_server_defines_handler_and_accesses_request | 1291 |
| service_client/task_003_mp3_db_service | FAILED | 2/7 | test_ros2_client_uses_node, test_ros2_client_creates_service_client, test_client_populates_request_fields, test_client_accesses_response_list, test_client_uses_response_to_drive_loop | 2246 |
| service_client/task_005_gazebo | FAILED | 4/7 | test_return_depends_on_service_response, test_response_used_in_return, test_multiple_service_calls_use_response | 1709 |
| service_client/task_006_gazebo_set_get_state | FAILED | 5/8 | test_03_retry_wait_for_service_with_shutdown_guard, test_05_full_service_call_chain_success_and_failure_handling, test_06_task_specific_subscriptions_combined | 2693 |
| service_client/task_007_navigation_make_plan | FAILED | 2/4 | test_03_requestmap_has_mutex_lock_guard, test_04_response_map_flow | 32472 |
| service_client/task_008_clear_costmaps | FAILED | 4/6 | test_equivalence_layer_names_semantics_obstacles_static_map, test_equivalence_gates_both_obstacles_and_static_map | 3040 |
| service_client/task_009_moveit_planning_scene | SUCCESS | 4/4 | None | 4559 |
| service_client/task_010_controller_manager | FAILED | 1/5 | test_uses_service_clients, test_start_stop_controllers_semantics, test_list_controllers_semantics, test_reload_libraries_semantics | 2131 |
| service_client/task_011_robot_services | FAILED | 2/6 | test_target_camera_driver_is_referenced, test_init_sets_up_configuration_interface, test_disable_auto_sets_both_flags_false, test_enable_auto_sets_both_flags_true | 1401 |
