# System Level Benchmark Report: PARAMETER_SERVER

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| parameter_server/task_001_basic_param | FAILED | 3/7 | test_no_ros1_master_xmlrpc, test_has_cache_map_and_subscribed_set, test_parameter_event_subscription_is_real, test_getImpl_has_cache_hit_shortcut_and_subscribe_on_miss | 16374 |
| parameter_server/task_002_dynamic_robot_configuration | FAILED | 5/6 | test_type_strictness_casting | 2721 |
| parameter_server/task_003_Mobile_robot_control | SUCCESS | 7/7 | None | 22091 |
| parameter_server/task_004_turtlrbot3_params | SUCCESS | 9/9 | None | 8737 |
| parameter_server/task_005_laser_scan_filter_params | FAILED | 5/6 | test_qos_and_topics | 3383 |
| parameter_server/task_006_dynamic_param_rqt_reconfigure | SUCCESS | 6/6 | None | 4307 |
| parameter_server/task_007_navigation_stack_config | FAILED | 4/7 | test_min_max_constraint_logic, test_internal_state_sync, test_no_legacy_artifacts | 30178 |
| parameter_server/task_008_gazebo_sim_param | FAILED | 5/8 | test_multicamera_suffix_preservation, test_standard_library_pointer_migration, test_time_source_logic | 13884 |
| parameter_server/task_009_fetch_arm_params | FAILED | 2/6 | test_ros2_param_declaration, test_snake_case_naming, test_logging_migration, test_tip_frame_validation | 33466 |
| parameter_server/task_010_sensor_fusion_params | FAILED | 4/6 | test_ros2_parameter_declaration, test_sensor_config_logic | 26316 |
| parameter_server/task_011_moveit_motion_params | FAILED | 4/6 | test_ros2_node_lifecycle_setup, test_planning_scene_availability | 7156 |
| parameter_server/task_012_px4_flight_params | FAILED | 5/8 | test_parameter_declaration_logic, test_dynamic_message_population_yaw, test_dynamic_message_population_position | 4636 |
| parameter_server/task_013_slam_mapping_params | FAILED | 4/7 | test_ros2_lifecycle_sequence, test_explicit_type_casting, test_member_variable_assignment | 16058 |
