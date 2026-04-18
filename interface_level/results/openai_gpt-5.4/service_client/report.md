# Benchmark Report: SERVICE_CLIENT

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| service_client/task_001_simple_service | SUCCESS | 8/8 | None | 750 |
| service_client/task_002_custom_srv | SUCCESS | 7/7 | None | 760 |
| service_client/task_003_mp3_db_service | FAILED | 3/7 | test_ros2_client_uses_node, test_ros2_client_creates_service_client, test_client_accesses_response_list, test_client_uses_response_to_drive_loop | 1584 |
| service_client/task_005_gazebo | FAILED | 5/7 | test_response_used_in_return, test_multiple_service_calls_use_response | 1386 |
| service_client/task_006_gazebo_set_get_state | FAILED | 7/8 | test_05_full_service_call_chain_success_and_failure_handling | 1565 |
| service_client/task_007_navigation_make_plan | FAILED | 3/4 | test_03_requestmap_has_mutex_lock_guard | 26741 |
| service_client/task_008_clear_costmaps | FAILED | 5/6 | test_equivalence_reset_distance_is_expressed | 2182 |
| service_client/task_009_moveit_planning_scene | SUCCESS | 4/4 | None | 3817 |
| service_client/task_010_controller_manager | FAILED | 2/5 | test_start_stop_controllers_semantics, test_list_controllers_semantics, test_reload_libraries_semantics | 1449 |
| service_client/task_011_robot_services | FAILED | 5/6 | test_target_camera_driver_is_referenced | 1011 |
