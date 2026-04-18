# Benchmark Report: PUBLISHER_SUBSCRIBER

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| publisher_subscriber/task_001_simple_pub_sub | FAILED | 1/2 | test_talker_ros2_translation | 1326 |
| publisher_subscriber/task_002_custom_msg_basic | SUCCESS | 2/2 | None | 1789 |
| publisher_subscriber/task_003_image_transport | SUCCESS | 3/3 | None | 1961 |
| publisher_subscriber/task_004_laser_scan_origin | FAILED | 2/3 | test_lidar_publisher_translation | 2763 |
| publisher_subscriber/task_005_ros1_ros2_bridge | SUCCESS | 1/1 | None | 765 |
| publisher_subscriber/task_006_multi_topic_synchronization | FAILED | 6/7 | test_oracle_4_two_image_subscribers | 1825 |
| publisher_subscriber/task_007_latched_publisher | FAILED | 2/3 | test_latched_behavior_for_new_subscriber | 962 |
| publisher_subscriber/task_009_nodelet_pubsub | FAILED | 5/9 | test_publisher_creation_ros2_style, test_subscription_creation_ros2_style, test_callback_uses_ros2_message_types, test_namespace_semantics_preserved | 1135 |
| publisher_subscriber/task_010_husky_stress_test | FAILED | 0/7 | test_translated_files_exist, test_no_ros1_artifacts_in_launch, test_robot_description_defined, test_robot_description_consumed_in_launch, test_imu_interface_present, test_gps_interface_present, test_sensor_update_rate_semantics | 0 |
