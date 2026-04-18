# Benchmark Report: PARAMETER_SERVER

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| parameter_server/task_001_basic_param | FAILED | 2/7 | test_no_ros1_master_xmlrpc, test_parameter_event_subscription_is_real, test_update_is_gated_by_subscribed_set_and_mutates_cache, test_invalidate_parent_walks_up_namespaces_and_erases_parents, test_getImpl_has_cache_hit_shortcut_and_subscribe_on_miss | 15694 |
| parameter_server/task_002_dynamic_robot_configuration | FAILED | 4/6 | test_default_value_integrity, test_type_strictness_casting | 3411 |
| parameter_server/task_003_Mobile_robot_control | FAILED | 5/7 | test_ros2_logging_style_compliance, test_absence_of_xmlrpc | 26984 |
| parameter_server/task_004_turtlrbot3_params | FAILED | 7/9 | test_physics_logic_preservation, test_value_extraction_style | 13156 |
| parameter_server/task_005_laser_scan_filter_params | FAILED | 4/6 | test_tf_filter_binding, test_qos_and_topics | 7003 |
| parameter_server/task_006_dynamic_param_rqt_reconfigure | SUCCESS | 6/6 | None | 5099 |
| parameter_server/task_007_navigation_stack_config | FAILED | 4/7 | test_min_max_constraint_logic, test_internal_state_sync, test_no_legacy_artifacts | 29712 |
| parameter_server/task_008_gazebo_sim_param | FAILED | 6/8 | test_standard_library_pointer_migration, test_time_source_logic | 18835 |
| parameter_server/task_009_fetch_arm_params | FAILED | 3/6 | test_ros2_param_declaration, test_logging_migration, test_tip_frame_validation | 30304 |
| parameter_server/task_010_sensor_fusion_params | FAILED | 5/6 | test_sensor_config_logic | 25300 |
| parameter_server/task_011_moveit_motion_params | FAILED | 4/6 | test_ros2_node_lifecycle_setup, test_async_execution_paradigm | 11598 |
| parameter_server/task_012_px4_flight_params | FAILED | 5/8 | test_parameter_declaration_logic, test_px4_msg_field_integrity, test_dynamic_message_population_position | 6168 |
| parameter_server/task_013_slam_mapping_params | FAILED | 6/7 | test_scanner_physical_validation_logic | 21059 |
