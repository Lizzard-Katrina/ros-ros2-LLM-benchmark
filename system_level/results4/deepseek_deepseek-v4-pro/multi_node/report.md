# System Level Benchmark Report: MULTI_NODE

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| multi_node/task_001_rosserial_python_integration | SUCCESS | 10/10 | None | 16700 |
| multi_node/task_002_sm_rosbag_recorder | SUCCESS | 7/7 | None | 13468 |
| multi_node/task_004_talker_relay_listener | SUCCESS | 8/8 | None | 4056 |
| multi_node/task_005_KIE_multiagent_model | SUCCESS | 11/11 | None | 29063 |
| multi_node/task_006_swarm_ros_bridge | FAILED | 4/8 | test_hpp_ros2_interface_migration, test_cpp_node_inheritance_usage, test_cpp_qos_usage, test_system_callback_consistency | 8452 |
| multi_node/task_007_aws_les | FAILED | 3/8 | test_member_variable_consistency, test_callback_signature_sync, test_inheritance_pattern, test_message_namespace_migration, test_ros1_leakage_cleanup | 1800 |
| multi_node/task_009_fleet_system | FAILED | 2/5 | test_time_struct_field_sync, test_header_schema_no_seq, test_no_ros1_naming_leakage | 5169 |
| multi_node/task_011_amr_interop | FAILED | 5/6 | test_no_ros1_terminology_leakage | 10966 |
| multi_node/task_012_multimaster_flie | FAILED | 6/8 | test_monitor_uses_succeed_helper, test_absence_of_hardcoded_names | 29309 |
