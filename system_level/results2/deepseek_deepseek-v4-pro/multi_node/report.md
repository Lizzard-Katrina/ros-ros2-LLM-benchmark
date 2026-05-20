# System Level Benchmark Report: MULTI_NODE

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| multi_node/task_001_rosserial_python_integration | FAILED | 8/10 | test_executor_spin, test_absence_of_global_node_init | 16400 |
| multi_node/task_002_sm_rosbag_recorder | SUCCESS | 7/7 | None | 13310 |
| multi_node/task_004_talker_relay_listener | SUCCESS | 8/8 | None | 4050 |
| multi_node/task_005_KIE_multiagent_model | SUCCESS | 11/11 | None | 28912 |
| multi_node/task_006_swarm_ros_bridge | FAILED | 4/8 | test_hpp_ros2_interface_migration, test_cpp_node_inheritance_usage, test_cpp_qos_usage, test_system_callback_consistency | 8523 |
| multi_node/task_007_aws_les | FAILED | 3/8 | test_member_variable_consistency, test_callback_signature_sync, test_inheritance_pattern, test_message_namespace_migration, test_ros1_leakage_cleanup | 1955 |
| multi_node/task_009_fleet_system | FAILED | 1/5 | test_time_struct_field_sync, test_header_schema_no_seq, test_namespace_conversion_logic, test_no_ros1_naming_leakage | 5046 |
| multi_node/task_011_amr_interop | FAILED | 5/6 | test_no_ros1_terminology_leakage | 10637 |
| multi_node/task_012_multimaster_flie | FAILED | 6/8 | test_monitor_uses_succeed_helper, test_absence_of_hardcoded_names | 29810 |
