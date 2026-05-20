# System Level Benchmark Report: PARAMETER_SERVER

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| parameter_server/task_001_basic_param | FAILED | 4/7 | test_parameter_event_subscription_is_real, test_invalidate_parent_walks_up_namespaces_and_erases_parents, test_getImpl_has_cache_hit_shortcut_and_subscribe_on_miss | 11466 |
| parameter_server/task_002_dynamic_robot_configuration | FAILED | 4/6 | test_default_value_integrity, test_type_strictness_casting | 1280 |
| parameter_server/task_003_Mobile_robot_control | SUCCESS | 7/7 | None | 17030 |
| parameter_server/task_004_turtlrbot3_params | FAILED | 7/9 | test_physics_logic_preservation, test_value_extraction_style | 5965 |
| parameter_server/task_005_laser_scan_filter_params | FAILED | 5/6 | test_qos_and_topics | 2345 |
| parameter_server/task_006_dynamic_param_rqt_reconfigure | SUCCESS | 6/6 | None | 3130 |
| parameter_server/task_007_navigation_stack_config | FAILED | 5/7 | test_min_max_constraint_logic, test_internal_state_sync | 24651 |
| parameter_server/task_008_gazebo_sim_param | FAILED | 5/8 | test_standard_library_pointer_migration, test_time_source_logic, test_std_function_migration | 11589 |
| parameter_server/task_009_fetch_arm_params | FAILED | 4/6 | test_logging_migration, test_tip_frame_validation | 27583 |
| parameter_server/task_010_sensor_fusion_params | FAILED | 4/6 | test_ros2_parameter_declaration, test_sensor_config_logic | 23542 |
| parameter_server/task_011_moveit_motion_params | FAILED | 2/6 | test_ros2_node_lifecycle_setup, test_moveit_cpp_namespace_accuracy, test_async_execution_paradigm, test_planning_scene_availability | 4697 |
| parameter_server/task_012_px4_flight_params | FAILED | 6/8 | test_parameter_declaration_logic, test_dynamic_message_population_position | 2893 |
| parameter_server/task_013_slam_mapping_params | FAILED | 5/7 | test_no_fake_nodehandle, test_member_variable_assignment | 15114 |
