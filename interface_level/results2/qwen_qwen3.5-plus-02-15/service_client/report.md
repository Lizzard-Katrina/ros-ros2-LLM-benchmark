# System Level Benchmark Report: SERVICE_CLIENT

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| service_client/task_001_simple_service | SUCCESS | 8/8 | None | 1045 |
| service_client/task_002_custom_srv | FAILED | 5/7 | test_ros2_server_uses_node_subclass, test_server_defines_handler_and_accesses_request | 1061 |
| service_client/task_003_mp3_db_service | FAILED | 2/7 | test_ros2_client_uses_node, test_ros2_client_creates_service_client, test_client_populates_request_fields, test_client_accesses_response_list, test_client_uses_response_to_drive_loop | 1539 |
| service_client/task_005_gazebo | FAILED | 4/7 | test_return_depends_on_service_response, test_response_used_in_return, test_multiple_service_calls_use_response | 1843 |
| service_client/task_006_gazebo_set_get_state | FAILED | 6/8 | test_03_retry_wait_for_service_with_shutdown_guard, test_05_full_service_call_chain_success_and_failure_handling | 1997 |
| service_client/task_007_navigation_make_plan | FAILED | 3/4 | test_03_requestmap_has_mutex_lock_guard | 34806 |
| service_client/task_008_clear_costmaps | FAILED | 4/6 | test_equivalence_layer_names_semantics_obstacles_static_map, test_equivalence_gates_both_obstacles_and_static_map | 2374 |
| service_client/task_009_moveit_planning_scene | SUCCESS | 4/4 | None | 4434 |
| service_client/task_010_controller_manager | FAILED | 4/5 | test_reload_libraries_semantics | 2324 |
| service_client/task_011_robot_services | FAILED | 2/6 | test_ros2_only_no_rospy_dynamic_reconfigure, test_target_camera_driver_is_referenced, test_parameters_are_used, test_init_sets_up_configuration_interface | 1079 |
