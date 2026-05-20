# System Level Benchmark Report: SENSOR_PROCESS

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| sensor_process/task_001_laser_obstacle_detection | FAILED | 3/4 | test_member_variable_consistency | 3613 |
| sensor_process/task_002_camera_edge_detection | SUCCESS | 6/6 | None | 907 |
| sensor_process/task_003_rgbd_pointcloud_generation | FAILED | 4/6 | test_intrinsic_scaling_math, test_dispatch_logic_16uc1_32fc1 | 4734 |
| sensor_process/task_004_imu_odometry_estimation | FAILED | 3/6 | test_3d_rotation_coupling, test_acceleration_3d_projection, test_eigen_optimization | 10059 |
| sensor_process/task_005_sonar_distance_estimation | FAILED | 1/6 | test_holonomic_kinematics, test_sonar_geometry_robust_v4, test_numerical_stability_epsilon, test_sonar_max_range_limit, test_sonar_y_mirroring_fix | 7910 |
| sensor_process/task_006_image_pipeline | FAILED | 5/9 | test_camera_info_scaling_logic, test_pointcloud2_field_setup, test_template_dispatch_completeness, test_radial_transform_update_check | 5550 |
| sensor_process/task_007_hector_slam | TRANSLATION_FAILED | 0/0 | None | 0 |
| sensor_process/task_008_darknet | FAILED | 3/6 | test_flexible_logging_macros, test_flexible_mutex_locking, test_timestamp_synchronization | 11373 |
| sensor_process/task_009_lio_sam | FAILED | 5/6 | test_callback_group_initialization | 40101 |
| sensor_process/task_010_radar_velocity_estimation | FAILED | 4/8 | test_ros2_logging_migration, test_ros2_clock_usage, test_mutex_lock_guard, test_zero_timestamp_handling | 4418 |
| sensor_process/task_011_camera_lidar_fusion | FAILED | 5/6 | test_namespace_full_compliance | 18707 |
