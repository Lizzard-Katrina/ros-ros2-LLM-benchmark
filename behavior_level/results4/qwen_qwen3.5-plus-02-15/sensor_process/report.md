# System Level Benchmark Report: SENSOR_PROCESS

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| sensor_process/task_001_laser_obstacle_detection | FAILED | 3/4 | test_member_variable_consistency | 2694 |
| sensor_process/task_002_camera_edge_detection | SUCCESS | 6/6 | None | 940 |
| sensor_process/task_003_rgbd_pointcloud_generation | FAILED | 4/6 | test_intrinsic_scaling_math, test_mandatory_kernel_call | 5214 |
| sensor_process/task_004_imu_odometry_estimation | FAILED | 2/6 | test_3d_rotation_coupling, test_acceleration_3d_projection, test_angular_singularity_mapping, test_eigen_optimization | 10754 |
| sensor_process/task_005_sonar_distance_estimation | FAILED | 1/6 | test_holonomic_kinematics, test_sonar_geometry_robust_v4, test_numerical_stability_epsilon, test_sonar_max_range_limit, test_sonar_y_mirroring_fix | 7919 |
| sensor_process/task_006_image_pipeline | FAILED | 6/9 | test_cv_bridge_exception_safety, test_template_dispatch_completeness, test_radial_transform_update_check | 4518 |
| sensor_process/task_007_hector_slam | FAILED | 6/7 | test_zero_ros1_leakage | 12762 |
| sensor_process/task_008_darknet | FAILED | 5/6 | test_flexible_logging_macros | 11522 |
| sensor_process/task_009_lio_sam | FAILED | 5/6 | test_callback_group_initialization | 39482 |
| sensor_process/task_010_radar_velocity_estimation | FAILED | 5/8 | test_ros2_logging_migration, test_ros2_clock_usage, test_zero_timestamp_handling | 4359 |
| sensor_process/task_011_camera_lidar_fusion | FAILED | 5/6 | test_tf2_geometry_msgs_include | 15141 |
