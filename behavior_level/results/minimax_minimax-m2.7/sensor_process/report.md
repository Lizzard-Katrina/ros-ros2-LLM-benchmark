# Benchmark Report: SENSOR_PROCESS

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| sensor_process/task_001_laser_obstacle_detection | FAILED | 3/4 | test_member_variable_consistency | 2604 |
| sensor_process/task_002_camera_edge_detection | SUCCESS | 6/6 | None | 1477 |
| sensor_process/task_003_rgbd_pointcloud_generation | FAILED | 3/6 | test_intrinsic_scaling_math, test_header_and_frame_sync, test_dispatch_logic_16uc1_32fc1 | 7982 |
| sensor_process/task_004_imu_odometry_estimation | FAILED | 3/6 | test_3d_rotation_coupling, test_acceleration_3d_projection, test_state_member_coverage | 6517 |
| sensor_process/task_005_sonar_distance_estimation | FAILED | 2/6 | test_sonar_geometry_robust_v4, test_numerical_stability_epsilon, test_sonar_max_range_limit, test_sonar_y_mirroring_fix | 7930 |
| sensor_process/task_006_image_pipeline | FAILED | 5/9 | test_camera_info_scaling_logic, test_cv_bridge_exception_safety, test_template_dispatch_completeness, test_radial_transform_update_check | 7142 |
| sensor_process/task_007_hector_slam | SUCCESS | 7/7 | None | 14348 |
| sensor_process/task_008_darknet | FAILED | 5/6 | test_flexible_logging_macros | 11402 |
| sensor_process/task_009_lio_sam | FAILED | 5/6 | test_callback_group_initialization | 38029 |
| sensor_process/task_010_radar_velocity_estimation | FAILED | 5/8 | test_ros2_logging_migration, test_ros2_clock_usage, test_zero_timestamp_handling | 4391 |
| sensor_process/task_011_camera_lidar_fusion | FAILED | 4/6 | test_tf2_geometry_msgs_include, test_namespace_full_compliance | 15657 |
