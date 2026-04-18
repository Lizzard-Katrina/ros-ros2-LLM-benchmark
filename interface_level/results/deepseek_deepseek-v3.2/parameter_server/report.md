# Benchmark Report: PARAMETER_SERVER

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| parameter_server/task_001_basic_param | FAILED | 5/7 | test_update_is_gated_by_subscribed_set_and_mutates_cache, test_getImpl_has_cache_hit_shortcut_and_subscribe_on_miss | 12494 |
| parameter_server/task_002_dynamic_robot_configuration | FAILED | 5/6 | test_type_strictness_casting | 1271 |
| parameter_server/task_003_Mobile_robot_control | FAILED | 5/7 | test_ros2_logging_style_compliance, test_exception_message_exact_match | 20661 |
| parameter_server/task_004_turtlrbot3_params | FAILED | 6/9 | test_api_constraint_compliance, test_physics_logic_preservation, test_value_extraction_style | 6860 |
| parameter_server/task_005_laser_scan_filter_params | FAILED | 4/6 | test_tf_filter_binding, test_qos_and_topics | 2535 |
| parameter_server/task_006_dynamic_param_rqt_reconfigure | SUCCESS | 6/6 | None | 3293 |
| parameter_server/task_007_navigation_stack_config | FAILED | 5/7 | test_min_max_constraint_logic, test_internal_state_sync | 26843 |
| parameter_server/task_008_gazebo_sim_param | FAILED | 5/8 | test_multicamera_suffix_preservation, test_standard_library_pointer_migration, test_time_source_logic | 12625 |
| parameter_server/task_009_fetch_arm_params | FAILED | 4/6 | test_snake_case_naming, test_logging_migration | 29205 |
| parameter_server/task_010_sensor_fusion_params | FAILED | 5/6 | test_ros2_parameter_declaration | 24557 |
| parameter_server/task_011_moveit_motion_params | FAILED | 1/6 | test_ros2_node_lifecycle_setup, test_moveit_cpp_namespace_accuracy, test_async_execution_paradigm, test_planning_scene_availability, test_message_namespace_migration | 4752 |
| parameter_server/task_012_px4_flight_params | FAILED | 6/8 | test_parameter_declaration_logic, test_dynamic_message_population_position | 2967 |
| parameter_server/task_013_slam_mapping_params | FAILED | 4/7 | test_scanner_physical_validation_logic, test_explicit_type_casting, test_member_variable_assignment | 17173 |
