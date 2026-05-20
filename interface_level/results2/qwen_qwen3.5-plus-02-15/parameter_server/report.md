# System Level Benchmark Report: PARAMETER_SERVER

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| parameter_server/task_001_basic_param | FAILED | 4/7 | test_parameter_event_subscription_is_real, test_update_is_gated_by_subscribed_set_and_mutates_cache, test_getImpl_has_cache_hit_shortcut_and_subscribe_on_miss | 14425 |
| parameter_server/task_002_dynamic_robot_configuration | FAILED | 4/6 | test_default_value_integrity, test_type_strictness_casting | 1393 |
| parameter_server/task_003_Mobile_robot_control | SUCCESS | 7/7 | None | 18477 |
| parameter_server/task_004_turtlrbot3_params | FAILED | 8/9 | test_physics_logic_preservation | 6776 |
| parameter_server/task_005_laser_scan_filter_params | FAILED | 3/6 | test_filter_chain_interface_usage, test_tf_filter_binding, test_qos_and_topics | 2646 |
| parameter_server/task_006_dynamic_param_rqt_reconfigure | SUCCESS | 6/6 | None | 3345 |
| parameter_server/task_007_navigation_stack_config | FAILED | 4/7 | test_min_max_constraint_logic, test_internal_state_sync, test_no_legacy_artifacts | 28140 |
| parameter_server/task_008_gazebo_sim_param | FAILED | 7/8 | test_time_source_logic | 13040 |
| parameter_server/task_009_fetch_arm_params | FAILED | 3/6 | test_ros2_param_declaration, test_snake_case_naming, test_logging_migration | 30026 |
| parameter_server/task_010_sensor_fusion_params | FAILED | 5/6 | test_sensor_config_logic | 22693 |
| parameter_server/task_011_moveit_motion_params | FAILED | 4/6 | test_ros2_node_lifecycle_setup, test_planning_scene_availability | 5146 |
| parameter_server/task_012_px4_flight_params | FAILED | 6/8 | test_parameter_declaration_logic, test_dynamic_message_population_position | 3157 |
| parameter_server/task_013_slam_mapping_params | FAILED | 4/7 | test_scanner_physical_validation_logic, test_no_fake_nodehandle, test_member_variable_assignment | 17385 |
