# Benchmark Report: PARAMETER_SERVER

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| parameter_server/task_001_basic_param | FAILED | 5/7 | test_update_is_gated_by_subscribed_set_and_mutates_cache, test_getImpl_has_cache_hit_shortcut_and_subscribe_on_miss | 21635 |
| parameter_server/task_002_dynamic_robot_configuration | FAILED | 4/6 | test_default_value_integrity, test_type_strictness_casting | 19925 |
| parameter_server/task_003_Mobile_robot_control | SUCCESS | 7/7 | None | 25511 |
| parameter_server/task_004_turtlrbot3_params | FAILED | 7/9 | test_physics_logic_preservation, test_value_extraction_style | 20613 |
| parameter_server/task_005_laser_scan_filter_params | FAILED | 4/6 | test_filter_chain_interface_usage, test_qos_and_topics | 13186 |
| parameter_server/task_006_dynamic_param_rqt_reconfigure | FAILED | 5/6 | test_sync_primitive | 10907 |
| parameter_server/task_007_navigation_stack_config | FAILED | 5/7 | test_min_max_constraint_logic, test_no_legacy_artifacts | 35723 |
| parameter_server/task_008_gazebo_sim_param | FAILED | 6/8 | test_multicamera_suffix_preservation, test_time_source_logic | 19535 |
| parameter_server/task_009_fetch_arm_params | FAILED | 2/6 | test_ros2_param_declaration, test_snake_case_naming, test_logging_migration, test_tip_frame_validation | 29669 |
| parameter_server/task_010_sensor_fusion_params | FAILED | 5/6 | test_ros2_parameter_declaration | 32364 |
| parameter_server/task_011_moveit_motion_params | FAILED | 4/6 | test_ros2_node_lifecycle_setup, test_async_execution_paradigm | 11164 |
| parameter_server/task_012_px4_flight_params | FAILED | 6/8 | test_parameter_declaration_logic, test_dynamic_message_population_position | 7845 |
| parameter_server/task_013_slam_mapping_params | FAILED | 3/7 | test_scanner_physical_validation_logic, test_explicit_type_casting, test_no_fake_nodehandle, test_member_variable_assignment | 23689 |
