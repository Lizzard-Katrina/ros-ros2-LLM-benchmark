# System Level Benchmark Report: MULTI_NODE

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| multi_node/task_001_rosserial_python_integration | FAILED | 7/10 | test_dependency_injection_linkage, test_executor_spin, test_absence_of_global_node_init | 16380 |
| multi_node/task_002_sm_rosbag_recorder | FAILED | 6/7 | test_recorder_qos_awareness | 12727 |
| multi_node/task_004_talker_relay_listener | SUCCESS | 8/8 | None | 4013 |
| multi_node/task_005_KIE_multiagent_model | FAILED | 8/11 | test_md_udp_protocol_preservation, test_md_variable_consistency, test_if_no_xmlrpc_usage | 22566 |
| multi_node/task_006_swarm_ros_bridge | FAILED | 4/8 | test_hpp_ros2_interface_migration, test_cpp_node_inheritance_usage, test_cpp_qos_usage, test_system_callback_consistency | 8796 |
| multi_node/task_007_aws_les | FAILED | 3/8 | test_member_variable_consistency, test_callback_signature_sync, test_inheritance_pattern, test_message_namespace_migration, test_ros1_leakage_cleanup | 1931 |
| multi_node/task_009_fleet_system | FAILED | 1/5 | test_time_struct_field_sync, test_header_schema_no_seq, test_namespace_conversion_logic, test_no_ros1_naming_leakage | 5068 |
| multi_node/task_011_amr_interop | FAILED | 4/6 | test_py_ros_to_mqtt_orchestration, test_no_ros1_terminology_leakage | 10743 |
| multi_node/task_012_multimaster_flie | FAILED | 2/8 | test_monitor_uses_succeed_helper, test_sync_uses_multicall, test_sync_loop_prevention, test_sync_filter_application, test_sync_preserves_remote_uri, test_absence_of_hardcoded_names | 22450 |
