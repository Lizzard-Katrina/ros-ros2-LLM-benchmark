# Benchmark Report: SENSOR_PROCESS

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| sensor_process/task_001_laser_obstacle_detection | FAILED | 3/4 | test_member_variable_consistency | 11435 |
| sensor_process/task_002_camera_edge_detection | SUCCESS | 6/6 | None | 4270 |
| sensor_process/task_003_rgbd_pointcloud_generation | FAILED | 4/6 | test_intrinsic_scaling_math, test_header_and_frame_sync | 15080 |
| sensor_process/task_004_imu_odometry_estimation | FAILED | 3/6 | test_3d_rotation_coupling, test_acceleration_3d_projection, test_state_member_coverage | 22469 |
| sensor_process/task_005_sonar_distance_estimation | FAILED | 2/6 | test_sonar_geometry_robust_v4, test_numerical_stability_epsilon, test_sonar_max_range_limit, test_sonar_y_mirroring_fix | 17920 |
| sensor_process/task_006_image_pipeline | FAILED | 6/9 | test_cv_bridge_exception_safety, test_template_dispatch_completeness, test_radial_transform_update_check | 13919 |
| sensor_process/task_007_hector_slam | FAILED | 6/7 | test_timestamp_preservation | 20334 |
| sensor_process/task_008_darknet | SUCCESS | 6/6 | None | 18405 |
| sensor_process/task_009_lio_sam | FAILED | 3/6 | test_callback_group_initialization, test_service_response_success_set, test_no_legacy_ros1_symbols | 47278 |
| sensor_process/task_010_radar_velocity_estimation | FAILED | 4/8 | test_ros2_logging_migration, test_ros2_clock_usage, test_mutex_lock_guard, test_zero_timestamp_handling | 18618 |
| sensor_process/task_011_camera_lidar_fusion | FAILED | 5/6 | test_tf2_geometry_msgs_include | 23464 |
