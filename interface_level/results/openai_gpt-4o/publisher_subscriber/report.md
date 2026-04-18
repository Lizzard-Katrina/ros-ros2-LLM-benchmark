# Benchmark Report: PUBLISHER_SUBSCRIBER

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| publisher_subscriber/task_001_simple_pub_sub | FAILED | 1/2 | test_listener_ros2_translation | 760 |
| publisher_subscriber/task_002_custom_msg_basic | SUCCESS | 2/2 | None | 894 |
| publisher_subscriber/task_003_image_transport | SUCCESS | 3/3 | None | 797 |
| publisher_subscriber/task_004_laser_scan_origin | FAILED | 1/3 | test_lidar_publisher_translation, test_lidar_subscriber_translation | 1330 |
| publisher_subscriber/task_005_ros1_ros2_bridge | SUCCESS | 1/1 | None | 341 |
| publisher_subscriber/task_006_multi_topic_synchronization | FAILED | 5/7 | test_oracle_2_stereo_sync_class, test_oracle_4_two_image_subscribers | 842 |
| publisher_subscriber/task_007_latched_publisher | FAILED | 2/3 | test_latched_behavior_for_new_subscriber | 457 |
| publisher_subscriber/task_009_nodelet_pubsub | FAILED | 6/9 | test_uses_ros2_headers_not_ros1, test_callback_uses_ros2_message_types, test_namespace_semantics_preserved | 520 |
| publisher_subscriber/task_010_husky_stress_test | FAILED | 0/7 | test_translated_files_exist, test_no_ros1_artifacts_in_launch, test_robot_description_defined, test_robot_description_consumed_in_launch, test_imu_interface_present, test_gps_interface_present, test_sensor_update_rate_semantics | 0 |
