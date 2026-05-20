# System Level Benchmark Report: PARAMETER_SERVER

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| parameter_server/task_001_basic_param | FAILED | 5/7 | test_parameter_event_subscription_is_real, test_update_is_gated_by_subscribed_set_and_mutates_cache | 11996 |
| parameter_server/task_002_dynamic_robot_configuration | FAILED | 4/6 | test_default_value_integrity, test_type_strictness_casting | 1477 |
| parameter_server/task_003_Mobile_robot_control | FAILED | 6/7 | test_cmd_vel_timeout_assignment | 18920 |
| parameter_server/task_004_turtlrbot3_params | FAILED | 8/9 | test_physics_logic_preservation | 6914 |
| parameter_server/task_005_laser_scan_filter_params | FAILED | 5/6 | test_tf_filter_binding | 2754 |
| parameter_server/task_006_dynamic_param_rqt_reconfigure | SUCCESS | 6/6 | None | 3421 |
| parameter_server/task_007_navigation_stack_config | FAILED | 4/7 | test_min_max_constraint_logic, test_internal_state_sync, test_no_legacy_artifacts | 28421 |
| parameter_server/task_008_gazebo_sim_param | FAILED | 5/8 | test_multicamera_suffix_preservation, test_standard_library_pointer_migration, test_time_source_logic | 13003 |
| parameter_server/task_009_fetch_arm_params | FAILED | 4/6 | test_ros2_param_declaration, test_logging_migration | 29376 |
| parameter_server/task_010_sensor_fusion_params | FAILED | 5/6 | test_ros2_parameter_declaration | 22836 |
| parameter_server/task_011_moveit_motion_params | FAILED | 4/6 | test_ros2_node_lifecycle_setup, test_moveit_cpp_namespace_accuracy | 5312 |
| parameter_server/task_012_px4_flight_params | FAILED | 5/8 | test_parameter_declaration_logic, test_dynamic_message_population_yaw, test_dynamic_message_population_position | 3155 |
| parameter_server/task_013_slam_mapping_params | FAILED | 5/7 | test_no_fake_nodehandle, test_member_variable_assignment | 18039 |
