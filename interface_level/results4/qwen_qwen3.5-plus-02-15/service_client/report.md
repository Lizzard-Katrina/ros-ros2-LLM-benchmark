# System Level Benchmark Report: SERVICE_CLIENT

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| service_client/task_001_simple_service | SUCCESS | 8/8 | None | 1045 |
| service_client/task_002_custom_srv | FAILED | 5/7 | test_ros2_server_uses_node_subclass, test_server_defines_handler_and_accesses_request | 1059 |
| service_client/task_003_mp3_db_service | FAILED | 3/7 | test_ros2_client_uses_node, test_ros2_client_creates_service_client, test_client_accesses_response_list, test_client_uses_response_to_drive_loop | 1624 |
| service_client/task_005_gazebo | FAILED | 4/7 | test_return_depends_on_service_response, test_response_used_in_return, test_multiple_service_calls_use_response | 1843 |
| service_client/task_006_gazebo_set_get_state | FAILED | 7/8 | test_05_full_service_call_chain_success_and_failure_handling | 2061 |
| service_client/task_007_navigation_make_plan | FAILED | 3/4 | test_03_requestmap_has_mutex_lock_guard | 35173 |
| service_client/task_008_clear_costmaps | SUCCESS | 6/6 | None | 2749 |
| service_client/task_009_moveit_planning_scene | SUCCESS | 4/4 | None | 4429 |
| service_client/task_010_controller_manager | FAILED | 3/5 | test_start_stop_controllers_semantics, test_reload_libraries_semantics | 2242 |
| service_client/task_011_robot_services | FAILED | 2/6 | test_ros2_only_no_rospy_dynamic_reconfigure, test_target_camera_driver_is_referenced, test_parameters_are_used, test_init_sets_up_configuration_interface | 1076 |
