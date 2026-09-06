# Benchmark Report: PARAMETER_SERVER

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| parameter_server/task_001_basic_param | FAILED | 3/7 | test_no_ros1_master_xmlrpc, test_has_cache_map_and_subscribed_set, test_parameter_event_subscription_is_real, test_getImpl_has_cache_hit_shortcut_and_subscribe_on_miss | 24629 |
| parameter_server/task_002_dynamic_robot_configuration | FAILED | 5/6 | test_type_strictness_casting | 6002 |
| parameter_server/task_003_Mobile_robot_control | SUCCESS | 7/7 | None | 36165 |
| parameter_server/task_004_turtlrbot3_params | SUCCESS | 9/9 | None | 11564 |
| parameter_server/task_005_laser_scan_filter_params | FAILED | 5/6 | test_qos_and_topics | 5547 |
| parameter_server/task_006_dynamic_param_rqt_reconfigure | SUCCESS | 6/6 | None | 6686 |
| parameter_server/task_007_navigation_stack_config | TRANSLATION_FAILED | 0/0 | None | 0 |
| parameter_server/task_008_gazebo_sim_param | FAILED | 5/8 | test_multicamera_suffix_preservation, test_standard_library_pointer_migration, test_time_source_logic | 26015 |
| parameter_server/task_009_fetch_arm_params | FAILED | 3/6 | test_ros2_param_declaration, test_logging_migration, test_tip_frame_validation | 43689 |
| parameter_server/task_010_sensor_fusion_params | FAILED | 4/6 | test_ros2_parameter_declaration, test_sensor_config_logic | 38934 |
| parameter_server/task_011_moveit_motion_params | FAILED | 4/6 | test_ros2_node_lifecycle_setup, test_planning_scene_availability | 13155 |
| parameter_server/task_012_px4_flight_params | FAILED | 5/8 | test_parameter_declaration_logic, test_dynamic_message_population_yaw, test_dynamic_message_population_position | 8306 |
| parameter_server/task_013_slam_mapping_params | FAILED | 4/7 | test_scanner_physical_validation_logic, test_explicit_type_casting, test_member_variable_assignment | 36917 |
