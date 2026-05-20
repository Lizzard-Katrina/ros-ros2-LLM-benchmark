# System Level Benchmark Report: MULTI_NODE

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| multi_node/task_001_rosserial_python_integration | FAILED | 9/10 | test_node_logger_usage | 14911 |
| multi_node/task_002_sm_rosbag_recorder | FAILED | 5/7 | test_recorder_qos_awareness, test_absence_of_legacy_ros1 | 11969 |
| multi_node/task_004_talker_relay_listener | SUCCESS | 8/8 | None | 3515 |
| multi_node/task_005_KIE_multiagent_model | FAILED | 10/11 | test_if_no_xmlrpc_usage | 23844 |
| multi_node/task_006_swarm_ros_bridge | FAILED | 5/8 | test_hpp_ros2_interface_migration, test_cpp_node_inheritance_usage, test_system_callback_consistency | 8344 |
| multi_node/task_007_aws_les | FAILED | 3/8 | test_member_variable_consistency, test_callback_signature_sync, test_inheritance_pattern, test_message_namespace_migration, test_ros1_leakage_cleanup | 1835 |
| multi_node/task_009_fleet_system | FAILED | 1/5 | test_time_struct_field_sync, test_header_schema_no_seq, test_namespace_conversion_logic, test_no_ros1_naming_leakage | 5281 |
| multi_node/task_011_amr_interop | FAILED | 4/6 | test_py_ros_to_mqtt_orchestration, test_no_ros1_terminology_leakage | 9335 |
| multi_node/task_012_multimaster_flie | FAILED | 7/8 | test_absence_of_hardcoded_names | 25091 |
