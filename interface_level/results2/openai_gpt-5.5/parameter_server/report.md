# System Level Benchmark Report: PARAMETER_SERVER

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| parameter_server/task_001_basic_param | FAILED | 2/7 | test_no_ros1_master_xmlrpc, test_has_cache_map_and_subscribed_set, test_parameter_event_subscription_is_real, test_invalidate_parent_walks_up_namespaces_and_erases_parents, test_getImpl_has_cache_hit_shortcut_and_subscribe_on_miss | 11762 |
| parameter_server/task_002_dynamic_robot_configuration | SUCCESS | 6/6 | None | 1345 |
| parameter_server/task_003_Mobile_robot_control | FAILED | 5/7 | test_absence_of_xmlrpc, test_no_ros1_nodehandle | 17304 |
| parameter_server/task_004_turtlrbot3_params | SUCCESS | 9/9 | None | 6098 |
| parameter_server/task_005_laser_scan_filter_params | FAILED | 5/6 | test_qos_and_topics | 2446 |
| parameter_server/task_006_dynamic_param_rqt_reconfigure | SUCCESS | 6/6 | None | 3166 |
| parameter_server/task_007_navigation_stack_config | FAILED | 6/7 | test_internal_state_sync | 24386 |
| parameter_server/task_008_gazebo_sim_param | FAILED | 5/8 | test_multicamera_suffix_preservation, test_standard_library_pointer_migration, test_time_source_logic | 11458 |
| parameter_server/task_009_fetch_arm_params | FAILED | 5/6 | test_ros2_param_declaration | 28996 |
| parameter_server/task_010_sensor_fusion_params | FAILED | 5/6 | test_sensor_config_logic | 23153 |
| parameter_server/task_011_moveit_motion_params | FAILED | 5/6 | test_ros2_node_lifecycle_setup | 4794 |
| parameter_server/task_012_px4_flight_params | FAILED | 5/8 | test_parameter_declaration_logic, test_parameter_value_retrieval, test_dynamic_message_population_position | 2807 |
| parameter_server/task_013_slam_mapping_params | FAILED | 4/7 | test_explicit_type_casting, test_no_fake_nodehandle, test_member_variable_assignment | 14538 |
