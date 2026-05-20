# System Level Benchmark Report: PUBLISHER_SUBSCRIBER

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| publisher_subscriber/task_001_simple_pub_sub | FAILED | 1/2 | test_talker_ros2_translation | 2337 |
| publisher_subscriber/task_002_custom_msg_basic | SUCCESS | 2/2 | None | 1972 |
| publisher_subscriber/task_003_image_transport | FAILED | 1/3 | test_camera_publisher_translation, test_camera_subscriber_translation | 2553 |
| publisher_subscriber/task_004_laser_scan_origin | SUCCESS | 3/3 | None | 2755 |
| publisher_subscriber/task_005_ros1_ros2_bridge | SUCCESS | 1/1 | None | 1124 |
| publisher_subscriber/task_006_multi_topic_synchronization | FAILED | 4/7 | test_oracle_2_stereo_sync_class, test_oracle_3_message_filters_synchronizer, test_oracle_4_two_image_subscribers | 1963 |
| publisher_subscriber/task_007_latched_publisher | FAILED | 2/3 | test_latched_behavior_for_new_subscriber | 1237 |
| publisher_subscriber/task_009_nodelet_pubsub | FAILED | 6/9 | test_uses_ros2_headers_not_ros1, test_ros2_parameter_usage, test_namespace_semantics_preserved | 1466 |
| publisher_subscriber/task_010_husky_stress_test | FAILED | 0/7 | test_translated_files_exist, test_no_ros1_artifacts_in_launch, test_robot_description_defined, test_robot_description_consumed_in_launch, test_imu_interface_present, test_gps_interface_present, test_sensor_update_rate_semantics | 533 |
