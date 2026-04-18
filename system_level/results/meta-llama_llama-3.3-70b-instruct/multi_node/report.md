# System Level Benchmark Report: MULTI_NODE

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| multi_node/task_001_rosserial_python_integration | FAILED | 9/10 | test_absence_of_global_node_init | 14722 |
| multi_node/task_002_sm_rosbag_recorder | FAILED | 5/7 | test_recorder_qos_awareness, test_talker_parameter_intent | 11799 |
| multi_node/task_004_talker_relay_listener | SUCCESS | 8/8 | None | 2410 |
| multi_node/task_005_KIE_multiagent_model | FAILED | 6/11 | test_md_ros2_publisher_definition, test_md_variable_consistency, test_if_no_xmlrpc_usage, test_if_host_filtering_logic, test_if_wait_loop_mechanism | 26054 |
| multi_node/task_006_swarm_ros_bridge | FAILED | 4/8 | test_hpp_ros2_interface_migration, test_hpp_no_ros1_leakage, test_cpp_qos_usage, test_system_callback_consistency | 6868 |
| multi_node/task_007_aws_les | FAILED | 3/8 | test_member_variable_consistency, test_callback_signature_sync, test_inheritance_pattern, test_message_namespace_migration, test_ros1_leakage_cleanup | 1912 |
| multi_node/task_009_fleet_system | FAILED | 2/5 | test_time_struct_field_sync, test_header_schema_no_seq, test_no_ros1_naming_leakage | 4704 |
| multi_node/task_011_amr_interop | FAILED | 4/6 | test_py_ros_to_mqtt_orchestration, test_no_ros1_terminology_leakage | 9243 |
| multi_node/task_012_multimaster_flie | FAILED | 1/8 | test_monitor_calls_master_api, test_monitor_populates_master_info, test_sync_uses_multicall, test_sync_loop_prevention, test_sync_filter_application, test_sync_preserves_remote_uri, test_absence_of_hardcoded_names | 21156 |
