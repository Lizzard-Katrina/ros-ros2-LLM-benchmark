# Benchmark Report: PARAMETER_SERVER

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| parameter_server/task_001_basic_param | FAILED | 4/7 | test_parameter_event_subscription_is_real, test_update_is_gated_by_subscribed_set_and_mutates_cache, test_getImpl_has_cache_hit_shortcut_and_subscribe_on_miss | 11297 |
| parameter_server/task_002_dynamic_robot_configuration | FAILED | 4/6 | test_default_value_integrity, test_type_strictness_casting | 1801 |
| parameter_server/task_003_Mobile_robot_control | FAILED | 5/7 | test_absence_of_xmlrpc, test_no_ros1_nodehandle | 18577 |
| parameter_server/task_004_turtlrbot3_params | FAILED | 6/9 | test_api_constraint_compliance, test_physics_logic_preservation, test_value_extraction_style | 8855 |
| parameter_server/task_005_laser_scan_filter_params | FAILED | 3/6 | test_filter_chain_interface_usage, test_tf_filter_binding, test_qos_and_topics | 3014 |
| parameter_server/task_006_dynamic_param_rqt_reconfigure | SUCCESS | 6/6 | None | 3556 |
| parameter_server/task_007_navigation_stack_config | FAILED | 5/7 | test_min_max_constraint_logic, test_no_legacy_artifacts | 25174 |
| parameter_server/task_008_gazebo_sim_param | FAILED | 5/8 | test_multicamera_suffix_preservation, test_standard_library_pointer_migration, test_time_source_logic | 12665 |
| parameter_server/task_009_fetch_arm_params | FAILED | 2/6 | test_ros2_param_declaration, test_snake_case_naming, test_logging_migration, test_moveit2_api_usage | 28377 |
| parameter_server/task_010_sensor_fusion_params | FAILED | 4/6 | test_ros2_parameter_declaration, test_sensor_config_logic | 12884 |
| parameter_server/task_011_moveit_motion_params | FAILED | 3/6 | test_ros2_node_lifecycle_setup, test_async_execution_paradigm, test_planning_scene_availability | 5834 |
| parameter_server/task_012_px4_flight_params | FAILED | 4/8 | test_parameter_declaration_logic, test_dynamic_message_population_yaw, test_px4_msg_field_integrity, test_dynamic_message_population_position | 4556 |
| parameter_server/task_013_slam_mapping_params | FAILED | 4/7 | test_explicit_type_casting, test_no_fake_nodehandle, test_member_variable_assignment | 15805 |
