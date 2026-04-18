# Benchmark Report: SENSOR_PROCESS

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| sensor_process/task_001_laser_obstacle_detection | FAILED | 3/4 | test_member_variable_consistency | 7964 |
| sensor_process/task_002_camera_edge_detection | SUCCESS | 6/6 | None | 4415 |
| sensor_process/task_003_rgbd_pointcloud_generation | FAILED | 5/6 | test_intrinsic_scaling_math | 8384 |
| sensor_process/task_004_imu_odometry_estimation | FAILED | 3/6 | test_3d_rotation_coupling, test_acceleration_3d_projection, test_eigen_optimization | 15417 |
| sensor_process/task_005_sonar_distance_estimation | FAILED | 2/6 | test_holonomic_kinematics, test_numerical_stability_epsilon, test_sonar_max_range_limit, test_sonar_y_mirroring_fix | 17089 |
| sensor_process/task_006_image_pipeline | FAILED | 7/9 | test_template_dispatch_completeness, test_ros2_unique_ptr_publish | 10691 |
| sensor_process/task_007_hector_slam | SUCCESS | 7/7 | None | 23795 |
| sensor_process/task_008_darknet | FAILED | 4/6 | test_flexible_logging_macros, test_flexible_mutex_locking | 16334 |
| sensor_process/task_009_lio_sam | FAILED | 5/6 | test_callback_group_initialization | 45110 |
| sensor_process/task_010_radar_velocity_estimation | FAILED | 5/8 | test_ros2_logging_migration, test_ros2_clock_usage, test_zero_timestamp_handling | 11577 |
| sensor_process/task_011_camera_lidar_fusion | TRANSLATION_FAILED | 0/0 | None | 0 |
