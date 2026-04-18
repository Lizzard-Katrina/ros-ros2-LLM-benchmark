# Benchmark Report: PARAMETER_SERVER

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| parameter_server/task_001_basic_param | FAILED | 2/7 | test_no_ros1_master_xmlrpc, test_has_cache_map_and_subscribed_set, test_parameter_event_subscription_is_real, test_invalidate_parent_walks_up_namespaces_and_erases_parents, test_getImpl_has_cache_hit_shortcut_and_subscribe_on_miss | 16023 |
| parameter_server/task_002_dynamic_robot_configuration | FAILED | 5/6 | test_type_strictness_casting | 2331 |
| parameter_server/task_003_Mobile_robot_control | FAILED | 5/7 | test_absence_of_xmlrpc, test_no_ros1_nodehandle | 17447 |
| parameter_server/task_004_turtlrbot3_params | FAILED | 7/9 | test_physics_logic_preservation, test_value_extraction_style | 7276 |
| parameter_server/task_005_laser_scan_filter_params | FAILED | 4/6 | test_qos_and_topics, test_deprecation_timer | 5031 |
| parameter_server/task_006_dynamic_param_rqt_reconfigure | SUCCESS | 6/6 | None | 3876 |
| parameter_server/task_007_navigation_stack_config | FAILED | 6/7 | test_no_legacy_artifacts | 25900 |
| parameter_server/task_008_gazebo_sim_param | FAILED | 5/8 | test_multicamera_suffix_preservation, test_standard_library_pointer_migration, test_time_source_logic | 25423 |
| parameter_server/task_009_fetch_arm_params | FAILED | 3/6 | test_ros2_param_declaration, test_logging_migration, test_tip_frame_validation | 39406 |
| parameter_server/task_010_sensor_fusion_params | FAILED | 5/6 | test_ros2_parameter_declaration | 22409 |
| parameter_server/task_011_moveit_motion_params | FAILED | 3/6 | test_ros2_node_lifecycle_setup, test_moveit_cpp_namespace_accuracy, test_planning_scene_availability | 7605 |
| parameter_server/task_012_px4_flight_params | FAILED | 6/8 | test_parameter_declaration_logic, test_dynamic_message_population_position | 5625 |
| parameter_server/task_013_slam_mapping_params | FAILED | 5/7 | test_no_fake_nodehandle, test_member_variable_assignment | 17715 |
