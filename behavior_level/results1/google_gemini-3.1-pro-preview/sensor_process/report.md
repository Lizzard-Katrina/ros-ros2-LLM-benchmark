# Benchmark Report: SENSOR_PROCESS

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| sensor_process/task_001_laser_obstacle_detection | SUCCESS | 4/4 | None | 8100 |
| sensor_process/task_002_camera_edge_detection | SUCCESS | 6/6 | None | 2382 |
| sensor_process/task_003_rgbd_pointcloud_generation | SUCCESS | 6/6 | None | 17509 |
| sensor_process/task_004_imu_odometry_estimation | FAILED | 2/6 | test_acceleration_3d_projection, test_angular_singularity_mapping, test_eigen_optimization, test_state_member_coverage | 0 |
| sensor_process/task_005_sonar_distance_estimation | FAILED | 1/6 | test_holonomic_kinematics, test_sonar_geometry_robust_v4, test_numerical_stability_epsilon, test_sonar_max_range_limit, test_sonar_y_mirroring_fix | 17758 |
| sensor_process/task_006_image_pipeline | FAILED | 7/9 | test_template_dispatch_completeness, test_radial_transform_update_check | 25281 |
| sensor_process/task_007_hector_slam | TRANSLATION_FAILED | 0/0 | None | 0 |
| sensor_process/task_008_darknet | SUCCESS | 6/6 | None | 32862 |
| sensor_process/task_009_lio_sam | FAILED | 5/6 | test_callback_group_initialization | 70102 |
| sensor_process/task_010_radar_velocity_estimation | FAILED | 4/8 | test_ros2_logging_migration, test_ros2_clock_usage, test_mutex_lock_guard, test_zero_timestamp_handling | 20420 |
| sensor_process/task_011_camera_lidar_fusion | SUCCESS | 6/6 | None | 28026 |
