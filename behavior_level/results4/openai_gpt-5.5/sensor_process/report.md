# System Level Benchmark Report: SENSOR_PROCESS

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| sensor_process/task_001_laser_obstacle_detection | FAILED | 3/4 | test_member_variable_consistency | 2801 |
| sensor_process/task_002_camera_edge_detection | SUCCESS | 6/6 | None | 858 |
| sensor_process/task_003_rgbd_pointcloud_generation | FAILED | 5/6 | test_intrinsic_scaling_math | 4500 |
| sensor_process/task_004_imu_odometry_estimation | FAILED | 4/6 | test_3d_rotation_coupling, test_acceleration_3d_projection | 9452 |
| sensor_process/task_005_sonar_distance_estimation | FAILED | 4/6 | test_numerical_stability_epsilon, test_sonar_max_range_limit | 6665 |
| sensor_process/task_006_image_pipeline | FAILED | 8/9 | test_template_dispatch_completeness | 5095 |
| sensor_process/task_007_hector_slam | FAILED | 5/7 | test_qos_durability_policy, test_zero_ros1_leakage | 11601 |
| sensor_process/task_008_darknet | FAILED | 4/6 | test_flexible_mutex_locking, test_timestamp_synchronization | 10927 |
| sensor_process/task_009_lio_sam | FAILED | 5/6 | test_callback_group_initialization | 38371 |
| sensor_process/task_010_radar_velocity_estimation | FAILED | 5/8 | test_ros2_logging_migration, test_ros2_clock_usage, test_zero_timestamp_handling | 3419 |
| sensor_process/task_011_camera_lidar_fusion | FAILED | 4/6 | test_tf2_geometry_msgs_include, test_namespace_full_compliance | 13820 |
