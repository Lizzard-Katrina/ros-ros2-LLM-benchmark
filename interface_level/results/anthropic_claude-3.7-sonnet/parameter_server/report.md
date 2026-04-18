# Benchmark Report: PARAMETER_SERVER

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| parameter_server/task_001_basic_param | FAILED | 4/7 | test_parameter_event_subscription_is_real, test_update_is_gated_by_subscribed_set_and_mutates_cache, test_invalidate_parent_walks_up_namespaces_and_erases_parents | 15019 |
| parameter_server/task_002_dynamic_robot_configuration | FAILED | 3/6 | test_correct_parameter_naming, test_default_value_integrity, test_type_strictness_casting | 1467 |
| parameter_server/task_003_Mobile_robot_control | FAILED | 6/7 | test_cmd_vel_timeout_assignment | 22378 |
| parameter_server/task_004_turtlrbot3_params | FAILED | 6/9 | test_api_constraint_compliance, test_physics_logic_preservation, test_value_extraction_style | 8035 |
| parameter_server/task_005_laser_scan_filter_params | FAILED | 4/6 | test_filter_chain_interface_usage, test_qos_and_topics | 2876 |
| parameter_server/task_006_dynamic_param_rqt_reconfigure | FAILED | 5/6 | test_deadlock_avoidance | 3928 |
| parameter_server/task_007_navigation_stack_config | FAILED | 6/7 | test_internal_state_sync | 37590 |
| parameter_server/task_008_gazebo_sim_param | FAILED | 7/8 | test_time_source_logic | 13794 |
| parameter_server/task_009_fetch_arm_params | FAILED | 5/6 | test_ros2_param_declaration | 36419 |
| parameter_server/task_010_sensor_fusion_params | FAILED | 4/6 | test_ros2_parameter_declaration, test_sensor_config_logic | 18873 |
| parameter_server/task_011_moveit_motion_params | FAILED | 3/6 | test_ros2_node_lifecycle_setup, test_moveit_cpp_namespace_accuracy, test_async_execution_paradigm | 5969 |
| parameter_server/task_012_px4_flight_params | FAILED | 6/8 | test_parameter_declaration_logic, test_dynamic_message_population_position | 3553 |
| parameter_server/task_013_slam_mapping_params | FAILED | 5/7 | test_explicit_type_casting, test_member_variable_assignment | 19577 |
