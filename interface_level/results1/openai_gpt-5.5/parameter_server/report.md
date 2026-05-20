# System Level Benchmark Report: PARAMETER_SERVER

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| parameter_server/task_001_basic_param | FAILED | 2/7 | test_no_ros1_master_xmlrpc, test_has_cache_map_and_subscribed_set, test_parameter_event_subscription_is_real, test_invalidate_parent_walks_up_namespaces_and_erases_parents, test_getImpl_has_cache_hit_shortcut_and_subscribe_on_miss | 11755 |
| parameter_server/task_002_dynamic_robot_configuration | SUCCESS | 6/6 | None | 1404 |
| parameter_server/task_003_Mobile_robot_control | SUCCESS | 7/7 | None | 17332 |
| parameter_server/task_004_turtlrbot3_params | SUCCESS | 9/9 | None | 6286 |
| parameter_server/task_005_laser_scan_filter_params | FAILED | 5/6 | test_qos_and_topics | 2345 |
| parameter_server/task_006_dynamic_param_rqt_reconfigure | SUCCESS | 6/6 | None | 3171 |
| parameter_server/task_007_navigation_stack_config | FAILED | 5/7 | test_min_max_constraint_logic, test_internal_state_sync | 23982 |
| parameter_server/task_008_gazebo_sim_param | FAILED | 6/8 | test_standard_library_pointer_migration, test_time_source_logic | 11625 |
| parameter_server/task_009_fetch_arm_params | FAILED | 4/6 | test_ros2_param_declaration, test_logging_migration | 26142 |
| parameter_server/task_010_sensor_fusion_params | FAILED | 5/6 | test_sensor_config_logic | 20861 |
| parameter_server/task_011_moveit_motion_params | FAILED | 5/6 | test_ros2_node_lifecycle_setup | 4826 |
| parameter_server/task_012_px4_flight_params | FAILED | 5/8 | test_parameter_declaration_logic, test_parameter_value_retrieval, test_dynamic_message_population_position | 2807 |
| parameter_server/task_013_slam_mapping_params | FAILED | 5/7 | test_no_fake_nodehandle, test_member_variable_assignment | 15331 |
