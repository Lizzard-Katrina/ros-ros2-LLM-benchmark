# System Level Benchmark Report: SENSOR_PROCESS

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| sensor_process/task_001_laser_obstacle_detection | SUCCESS | 4/4 | None | 3394 |
| sensor_process/task_002_camera_edge_detection | SUCCESS | 6/6 | None | 1146 |
| sensor_process/task_003_rgbd_pointcloud_generation | SUCCESS | 6/6 | None | 5930 |
| sensor_process/task_004_imu_odometry_estimation | FAILED | 4/6 | test_3d_rotation_coupling, test_acceleration_3d_projection | 12636 |
| sensor_process/task_005_sonar_distance_estimation | FAILED | 4/6 | test_numerical_stability_epsilon, test_sonar_max_range_limit | 9120 |
| sensor_process/task_006_image_pipeline | SUCCESS | 9/9 | None | 5992 |
| sensor_process/task_007_hector_slam | SUCCESS | 7/7 | None | 16341 |
| sensor_process/task_008_darknet | FAILED | 4/6 | test_flexible_logging_macros, test_flexible_mutex_locking | 14765 |
| sensor_process/task_009_lio_sam | FAILED | 4/6 | test_callback_group_initialization, test_service_response_success_set | 50582 |
| sensor_process/task_010_radar_velocity_estimation | FAILED | 4/8 | test_ros2_logging_migration, test_ros2_clock_usage, test_mutex_lock_guard, test_zero_timestamp_handling | 5223 |
| sensor_process/task_011_camera_lidar_fusion | FAILED | 4/6 | test_tf2_geometry_msgs_include, test_ukf_pipeline_integrity | 19520 |
