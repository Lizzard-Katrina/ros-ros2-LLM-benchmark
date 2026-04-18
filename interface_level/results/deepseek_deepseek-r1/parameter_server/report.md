# Benchmark Report: PARAMETER_SERVER

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| parameter_server/task_001_basic_param | FAILED | 0/7 | test_no_ros1_master_xmlrpc, test_has_cache_map_and_subscribed_set, test_parameter_event_subscription_is_real, test_update_is_gated_by_subscribed_set_and_mutates_cache, test_invalidate_parent_walks_up_namespaces_and_erases_parents, test_getImpl_has_cache_hit_shortcut_and_subscribe_on_miss, test_event_callback_calls_update_or_equivalent | 41997 |
| parameter_server/task_002_dynamic_robot_configuration | FAILED | 1/6 | test_uses_ros2_declare_parameter, test_enforces_vector_int64_structure, test_correct_parameter_naming, test_default_value_integrity, test_type_strictness_casting | 38283 |
| parameter_server/task_003_Mobile_robot_control | FAILED | 5/7 | test_cmd_vel_timeout_assignment, test_exception_message_exact_match | 20828 |
| parameter_server/task_004_turtlrbot3_params | FAILED | 7/9 | test_physics_logic_preservation, test_value_extraction_style | 10944 |
| parameter_server/task_005_laser_scan_filter_params | FAILED | 4/6 | test_tf_filter_binding, test_qos_and_topics | 4057 |
| parameter_server/task_006_dynamic_param_rqt_reconfigure | FAILED | 5/6 | test_sync_primitive | 5059 |
| parameter_server/task_007_navigation_stack_config | FAILED | 5/7 | test_min_max_constraint_logic, test_internal_state_sync | 27774 |
| parameter_server/task_008_gazebo_sim_param | FAILED | 1/8 | test_node_factory_migration, test_parameter_declaration_style, test_multicamera_suffix_preservation, test_standard_library_pointer_migration, test_logging_macros_migration, test_time_source_logic, test_std_function_migration | 39690 |
| parameter_server/task_009_fetch_arm_params | FAILED | 4/6 | test_snake_case_naming, test_logging_migration | 31107 |
| parameter_server/task_010_sensor_fusion_params | FAILED | 5/6 | test_ros2_parameter_declaration | 16911 |
| parameter_server/task_011_moveit_motion_params | FAILED | 3/6 | test_ros2_node_lifecycle_setup, test_async_execution_paradigm, test_planning_scene_availability | 6060 |
| parameter_server/task_012_px4_flight_params | FAILED | 6/8 | test_parameter_declaration_logic, test_dynamic_message_population_position | 3859 |
| parameter_server/task_013_slam_mapping_params | FAILED | 4/7 | test_scanner_physical_validation_logic, test_explicit_type_casting, test_member_variable_assignment | 18311 |
