# Benchmark Report: PARAMETER_SERVER

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| parameter_server/task_001_basic_param | FAILED | 4/7 | test_no_ros1_master_xmlrpc, test_invalidate_parent_walks_up_namespaces_and_erases_parents, test_getImpl_has_cache_hit_shortcut_and_subscribe_on_miss | 17585 |
| parameter_server/task_002_dynamic_robot_configuration | FAILED | 3/6 | test_correct_parameter_naming, test_default_value_integrity, test_type_strictness_casting | 1316 |
| parameter_server/task_003_Mobile_robot_control | FAILED | 5/7 | test_cmd_vel_timeout_assignment, test_absence_of_xmlrpc | 21925 |
| parameter_server/task_004_turtlrbot3_params | FAILED | 7/9 | test_physics_logic_preservation, test_value_extraction_style | 7279 |
| parameter_server/task_005_laser_scan_filter_params | FAILED | 5/6 | test_qos_and_topics | 2555 |
| parameter_server/task_006_dynamic_param_rqt_reconfigure | FAILED | 5/6 | test_sync_primitive | 3644 |
| parameter_server/task_007_navigation_stack_config | FAILED | 4/7 | test_min_max_constraint_logic, test_internal_state_sync, test_no_legacy_artifacts | 38132 |
| parameter_server/task_008_gazebo_sim_param | FAILED | 5/8 | test_multicamera_suffix_preservation, test_standard_library_pointer_migration, test_std_function_migration | 16165 |
| parameter_server/task_009_fetch_arm_params | FAILED | 4/6 | test_snake_case_naming, test_logging_migration | 32360 |
| parameter_server/task_010_sensor_fusion_params | FAILED | 4/6 | test_ros2_parameter_declaration, test_sensor_config_logic | 30579 |
| parameter_server/task_011_moveit_motion_params | FAILED | 3/6 | test_ros2_node_lifecycle_setup, test_async_execution_paradigm, test_planning_scene_availability | 5392 |
| parameter_server/task_012_px4_flight_params | FAILED | 6/8 | test_parameter_declaration_logic, test_dynamic_message_population_position | 3444 |
| parameter_server/task_013_slam_mapping_params | FAILED | 4/7 | test_explicit_type_casting, test_no_fake_nodehandle, test_member_variable_assignment | 18596 |
