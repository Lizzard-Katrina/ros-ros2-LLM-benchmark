# Benchmark Report: SERVICE_CLIENT

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| service_client/task_001_simple_service | FAILED | 7/8 | test_client_uses_rclpy_and_node | 1695 |
| service_client/task_002_custom_srv | FAILED | 6/7 | test_server_defines_handler_and_accesses_request | 1816 |
| service_client/task_003_mp3_db_service | FAILED | 2/7 | test_ros2_client_uses_node, test_ros2_client_creates_service_client, test_client_populates_request_fields, test_client_accesses_response_list, test_client_uses_response_to_drive_loop | 1264 |
| service_client/task_005_gazebo | FAILED | 4/7 | test_return_depends_on_service_response, test_response_used_in_return, test_multiple_service_calls_use_response | 2975 |
| service_client/task_006_gazebo_set_get_state | FAILED | 7/8 | test_05_full_service_call_chain_success_and_failure_handling | 4537 |
| service_client/task_007_navigation_make_plan | FAILED | 3/4 | test_03_requestmap_has_mutex_lock_guard | 33760 |
| service_client/task_008_clear_costmaps | FAILED | 0/6 | test_no_ros1_core_api_remnants, test_testCountLethal_calls_testClearBehavior, test_testClearBehavior_exists_and_not_stub, test_equivalence_reset_distance_is_expressed, test_equivalence_layer_names_semantics_obstacles_static_map, test_equivalence_gates_both_obstacles_and_static_map | 34699 |
| service_client/task_009_moveit_planning_scene | FAILED | 3/4 | test_category_3_minimal_diff_hygiene_reset_and_not_keep_in_world | 5541 |
| service_client/task_010_controller_manager | FAILED | 4/5 | test_reload_libraries_semantics | 4072 |
| service_client/task_011_robot_services | FAILED | 2/6 | test_target_camera_driver_is_referenced, test_init_sets_up_configuration_interface, test_disable_auto_sets_both_flags_false, test_enable_auto_sets_both_flags_true | 3985 |
