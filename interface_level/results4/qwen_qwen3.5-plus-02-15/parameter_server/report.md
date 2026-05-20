# System Level Benchmark Report: PARAMETER_SERVER

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| parameter_server/task_001_basic_param | FAILED | 5/7 | test_has_cache_map_and_subscribed_set, test_update_is_gated_by_subscribed_set_and_mutates_cache | 17498 |
| parameter_server/task_002_dynamic_robot_configuration | FAILED | 4/6 | test_correct_parameter_naming, test_default_value_integrity | 1386 |
| parameter_server/task_003_Mobile_robot_control | SUCCESS | 7/7 | None | 18397 |
| parameter_server/task_004_turtlrbot3_params | FAILED | 8/9 | test_physics_logic_preservation | 6716 |
| parameter_server/task_005_laser_scan_filter_params | FAILED | 4/6 | test_filter_chain_interface_usage, test_qos_and_topics | 2625 |
| parameter_server/task_006_dynamic_param_rqt_reconfigure | SUCCESS | 6/6 | None | 3345 |
| parameter_server/task_007_navigation_stack_config | FAILED | 5/7 | test_min_max_constraint_logic, test_no_legacy_artifacts | 27618 |
| parameter_server/task_008_gazebo_sim_param | FAILED | 7/8 | test_time_source_logic | 13045 |
| parameter_server/task_009_fetch_arm_params | FAILED | 4/6 | test_logging_migration, test_tip_frame_validation | 29837 |
| parameter_server/task_010_sensor_fusion_params | FAILED | 4/6 | test_ros2_parameter_declaration, test_sensor_config_logic | 22694 |
| parameter_server/task_011_moveit_motion_params | FAILED | 5/6 | test_ros2_node_lifecycle_setup | 5145 |
| parameter_server/task_012_px4_flight_params | FAILED | 7/8 | test_dynamic_message_population_position | 3166 |
| parameter_server/task_013_slam_mapping_params | FAILED | 5/7 | test_no_fake_nodehandle, test_member_variable_assignment | 17677 |
