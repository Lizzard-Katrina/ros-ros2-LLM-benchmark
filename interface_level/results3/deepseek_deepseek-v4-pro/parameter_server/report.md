# System Level Benchmark Report: PARAMETER_SERVER

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| parameter_server/task_001_basic_param | FAILED | 5/7 | test_parameter_event_subscription_is_real, test_update_is_gated_by_subscribed_set_and_mutates_cache | 12798 |
| parameter_server/task_002_dynamic_robot_configuration | FAILED | 4/6 | test_default_value_integrity, test_type_strictness_casting | 1484 |
| parameter_server/task_003_Mobile_robot_control | FAILED | 4/7 | test_exception_message_exact_match, test_covariance_vector_type, test_size_logic_check | 12912 |
| parameter_server/task_004_turtlrbot3_params | FAILED | 7/9 | test_physics_logic_preservation, test_value_extraction_style | 6843 |
| parameter_server/task_005_laser_scan_filter_params | FAILED | 5/6 | test_tf_filter_binding | 2732 |
| parameter_server/task_006_dynamic_param_rqt_reconfigure | SUCCESS | 6/6 | None | 3517 |
| parameter_server/task_007_navigation_stack_config | FAILED | 1/7 | test_parameter_declaration_logic, test_callback_binding_logic, test_min_max_constraint_logic, test_internal_state_sync, test_no_legacy_artifacts, test_callback_return_type | 26088 |
| parameter_server/task_008_gazebo_sim_param | FAILED | 5/8 | test_multicamera_suffix_preservation, test_standard_library_pointer_migration, test_time_source_logic | 12718 |
| parameter_server/task_009_fetch_arm_params | FAILED | 4/6 | test_ros2_param_declaration, test_logging_migration | 29535 |
| parameter_server/task_010_sensor_fusion_params | FAILED | 5/6 | test_ros2_parameter_declaration | 22884 |
| parameter_server/task_011_moveit_motion_params | FAILED | 4/6 | test_ros2_node_lifecycle_setup, test_async_execution_paradigm | 5386 |
| parameter_server/task_012_px4_flight_params | FAILED | 5/8 | test_parameter_declaration_logic, test_dynamic_message_population_yaw, test_dynamic_message_population_position | 3160 |
| parameter_server/task_013_slam_mapping_params | FAILED | 4/7 | test_scanner_physical_validation_logic, test_no_fake_nodehandle, test_member_variable_assignment | 17948 |
