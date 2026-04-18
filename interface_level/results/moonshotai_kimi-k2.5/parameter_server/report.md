# Benchmark Report: PARAMETER_SERVER

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| parameter_server/task_001_basic_param | FAILED | 6/7 | test_update_is_gated_by_subscribed_set_and_mutates_cache | 17270 |
| parameter_server/task_002_dynamic_robot_configuration | FAILED | 1/6 | test_uses_ros2_declare_parameter, test_enforces_vector_int64_structure, test_correct_parameter_naming, test_default_value_integrity, test_type_strictness_casting | 4247 |
| parameter_server/task_003_Mobile_robot_control | SUCCESS | 7/7 | None | 27710 |
| parameter_server/task_004_turtlrbot3_params | FAILED | 6/9 | test_api_constraint_compliance, test_physics_logic_preservation, test_value_extraction_style | 15069 |
| parameter_server/task_005_laser_scan_filter_params | FAILED | 4/6 | test_tf_filter_binding, test_qos_and_topics | 8155 |
| parameter_server/task_006_dynamic_param_rqt_reconfigure | SUCCESS | 6/6 | None | 4325 |
| parameter_server/task_007_navigation_stack_config | FAILED | 5/7 | test_min_max_constraint_logic, test_no_legacy_artifacts | 42391 |
| parameter_server/task_008_gazebo_sim_param | FAILED | 5/8 | test_multicamera_suffix_preservation, test_standard_library_pointer_migration, test_time_source_logic | 22346 |
| parameter_server/task_009_fetch_arm_params | FAILED | 4/6 | test_ros2_param_declaration, test_tip_frame_validation | 32568 |
| parameter_server/task_010_sensor_fusion_params | FAILED | 4/6 | test_ros2_parameter_declaration, test_sensor_config_logic | 16614 |
| parameter_server/task_011_moveit_motion_params | FAILED | 3/6 | test_ros2_node_lifecycle_setup, test_async_execution_paradigm, test_planning_scene_availability | 9331 |
| parameter_server/task_012_px4_flight_params | FAILED | 5/8 | test_parameter_declaration_logic, test_dynamic_message_population_yaw, test_dynamic_message_population_position | 5540 |
| parameter_server/task_013_slam_mapping_params | FAILED | 3/7 | test_scanner_physical_validation_logic, test_explicit_type_casting, test_no_fake_nodehandle, test_member_variable_assignment | 26337 |
