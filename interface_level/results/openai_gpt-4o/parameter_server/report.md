# Benchmark Report: PARAMETER_SERVER

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| parameter_server/task_001_basic_param | FAILED | 3/7 | test_parameter_event_subscription_is_real, test_update_is_gated_by_subscribed_set_and_mutates_cache, test_invalidate_parent_walks_up_namespaces_and_erases_parents, test_getImpl_has_cache_hit_shortcut_and_subscribe_on_miss | 10074 |
| parameter_server/task_002_dynamic_robot_configuration | FAILED | 3/6 | test_uses_ros2_declare_parameter, test_default_value_integrity, test_type_strictness_casting | 1120 |
| parameter_server/task_003_Mobile_robot_control | SUCCESS | 7/7 | None | 16819 |
| parameter_server/task_004_turtlrbot3_params | FAILED | 6/9 | test_api_constraint_compliance, test_physics_logic_preservation, test_value_extraction_style | 6202 |
| parameter_server/task_005_laser_scan_filter_params | FAILED | 4/6 | test_filter_chain_interface_usage, test_qos_and_topics | 2141 |
| parameter_server/task_006_dynamic_param_rqt_reconfigure | SUCCESS | 6/6 | None | 2908 |
| parameter_server/task_007_navigation_stack_config | FAILED | 5/7 | test_min_max_constraint_logic, test_no_legacy_artifacts | 24809 |
| parameter_server/task_008_gazebo_sim_param | FAILED | 5/8 | test_multicamera_suffix_preservation, test_standard_library_pointer_migration, test_time_source_logic | 10196 |
| parameter_server/task_009_fetch_arm_params | FAILED | 2/6 | test_ros2_param_declaration, test_logging_migration, test_moveit2_api_usage, test_tip_frame_validation | 26783 |
| parameter_server/task_010_sensor_fusion_params | FAILED | 5/6 | test_sensor_config_logic | 20357 |
| parameter_server/task_011_moveit_motion_params | FAILED | 3/6 | test_ros2_node_lifecycle_setup, test_moveit_cpp_namespace_accuracy, test_async_execution_paradigm | 4505 |
| parameter_server/task_012_px4_flight_params | FAILED | 4/8 | test_parameter_declaration_logic, test_dynamic_message_population_yaw, test_px4_msg_field_integrity, test_dynamic_message_population_position | 2502 |
| parameter_server/task_013_slam_mapping_params | FAILED | 1/7 | test_ros2_lifecycle_sequence, test_scanner_physical_validation_logic, test_explicit_type_casting, test_no_fake_nodehandle, test_slam_default_values, test_member_variable_assignment | 14644 |
