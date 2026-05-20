# System Level Benchmark Report: SENSOR_PROCESS

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| sensor_process/task_001_laser_obstacle_detection | FAILED | 3/4 | test_member_variable_consistency | 2784 |
| sensor_process/task_002_camera_edge_detection | SUCCESS | 6/6 | None | 860 |
| sensor_process/task_003_rgbd_pointcloud_generation | FAILED | 5/6 | test_intrinsic_scaling_math | 4565 |
| sensor_process/task_004_imu_odometry_estimation | FAILED | 5/6 | test_acceleration_3d_projection | 9371 |
| sensor_process/task_005_sonar_distance_estimation | FAILED | 4/6 | test_numerical_stability_epsilon, test_sonar_max_range_limit | 6762 |
| sensor_process/task_006_image_pipeline | FAILED | 8/9 | test_template_dispatch_completeness | 5267 |
| sensor_process/task_007_hector_slam | SUCCESS | 7/7 | None | 11851 |
| sensor_process/task_008_darknet | FAILED | 5/6 | test_timestamp_synchronization | 11101 |
| sensor_process/task_009_lio_sam | FAILED | 5/6 | test_callback_group_initialization | 36633 |
| sensor_process/task_010_radar_velocity_estimation | FAILED | 4/8 | test_ros2_logging_migration, test_ros2_clock_usage, test_mutex_lock_guard, test_zero_timestamp_handling | 3488 |
| sensor_process/task_011_camera_lidar_fusion | FAILED | 4/6 | test_tf2_geometry_msgs_include, test_namespace_full_compliance | 13748 |
