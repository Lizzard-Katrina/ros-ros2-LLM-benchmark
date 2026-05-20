# System Level Benchmark Report: SENSOR_PROCESS

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| sensor_process/task_001_laser_obstacle_detection | SUCCESS | 4/4 | None | 3905 |
| sensor_process/task_002_camera_edge_detection | SUCCESS | 6/6 | None | 1943 |
| sensor_process/task_003_rgbd_pointcloud_generation | FAILED | 5/6 | test_mandatory_kernel_call | 8407 |
| sensor_process/task_004_imu_odometry_estimation | FAILED | 3/6 | test_3d_rotation_coupling, test_acceleration_3d_projection, test_angular_singularity_mapping | 13091 |
| sensor_process/task_005_sonar_distance_estimation | FAILED | 3/6 | test_numerical_stability_epsilon, test_sonar_max_range_limit, test_sonar_y_mirroring_fix | 9235 |
| sensor_process/task_006_image_pipeline | FAILED | 7/9 | test_template_dispatch_completeness, test_radial_transform_update_check | 5354 |
| sensor_process/task_007_hector_slam | FAILED | 6/7 | test_ros2_logging_and_time | 15010 |
| sensor_process/task_008_darknet | FAILED | 4/6 | test_flexible_logging_macros, test_flexible_mutex_locking | 15079 |
| sensor_process/task_009_lio_sam | FAILED | 5/6 | test_callback_group_initialization | 43466 |
| sensor_process/task_010_radar_velocity_estimation | FAILED | 4/8 | test_ros2_logging_migration, test_ros2_clock_usage, test_mutex_lock_guard, test_zero_timestamp_handling | 5791 |
| sensor_process/task_011_camera_lidar_fusion | FAILED | 5/6 | test_namespace_full_compliance | 18929 |
