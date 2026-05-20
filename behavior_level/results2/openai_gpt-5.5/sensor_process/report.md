# System Level Benchmark Report: SENSOR_PROCESS

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| sensor_process/task_001_laser_obstacle_detection | FAILED | 3/4 | test_member_variable_consistency | 2906 |
| sensor_process/task_002_camera_edge_detection | SUCCESS | 6/6 | None | 852 |
| sensor_process/task_003_rgbd_pointcloud_generation | FAILED | 5/6 | test_intrinsic_scaling_math | 4561 |
| sensor_process/task_004_imu_odometry_estimation | FAILED | 3/6 | test_3d_rotation_coupling, test_acceleration_3d_projection, test_angular_singularity_mapping | 9886 |
| sensor_process/task_005_sonar_distance_estimation | FAILED | 4/6 | test_numerical_stability_epsilon, test_sonar_max_range_limit | 6695 |
| sensor_process/task_006_image_pipeline | FAILED | 7/9 | test_cv_bridge_exception_safety, test_template_dispatch_completeness | 4876 |
| sensor_process/task_007_hector_slam | FAILED | 6/7 | test_zero_ros1_leakage | 11487 |
| sensor_process/task_008_darknet | FAILED | 5/6 | test_timestamp_synchronization | 11049 |
| sensor_process/task_009_lio_sam | FAILED | 4/6 | test_callback_group_initialization, test_service_response_success_set | 36505 |
| sensor_process/task_010_radar_velocity_estimation | FAILED | 4/8 | test_ros2_logging_migration, test_ros2_clock_usage, test_mutex_lock_guard, test_zero_timestamp_handling | 3408 |
| sensor_process/task_011_camera_lidar_fusion | FAILED | 4/6 | test_tf2_geometry_msgs_include, test_namespace_full_compliance | 13820 |
