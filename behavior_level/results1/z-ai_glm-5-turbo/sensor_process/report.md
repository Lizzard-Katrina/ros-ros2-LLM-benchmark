# Benchmark Report: SENSOR_PROCESS

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| sensor_process/task_001_laser_obstacle_detection | FAILED | 3/4 | test_member_variable_consistency | 11379 |
| sensor_process/task_002_camera_edge_detection | SUCCESS | 6/6 | None | 1860 |
| sensor_process/task_003_rgbd_pointcloud_generation | FAILED | 5/6 | test_intrinsic_scaling_math | 5764 |
| sensor_process/task_004_imu_odometry_estimation | FAILED | 4/6 | test_3d_rotation_coupling, test_acceleration_3d_projection | 15317 |
| sensor_process/task_005_sonar_distance_estimation | FAILED | 3/6 | test_holonomic_kinematics, test_numerical_stability_epsilon, test_sonar_max_range_limit | 12830 |
| sensor_process/task_006_image_pipeline | FAILED | 8/9 | test_template_dispatch_completeness | 8361 |
| sensor_process/task_007_hector_slam | SUCCESS | 7/7 | None | 22132 |
| sensor_process/task_008_darknet | FAILED | 5/6 | test_flexible_mutex_locking | 19322 |
| sensor_process/task_009_lio_sam | FAILED | 3/6 | test_callback_group_initialization, test_service_response_success_set, test_no_legacy_ros1_symbols | 43699 |
| sensor_process/task_010_radar_velocity_estimation | FAILED | 6/8 | test_ros2_clock_usage, test_zero_timestamp_handling | 10786 |
| sensor_process/task_011_camera_lidar_fusion | FAILED | 4/6 | test_tf2_geometry_msgs_include, test_namespace_full_compliance | 9894 |
