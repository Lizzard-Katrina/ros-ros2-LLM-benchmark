# Benchmark Report: SENSOR_PROCESS

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| sensor_process/task_001_laser_obstacle_detection | FAILED | 2/4 | test_no_lifecycle_hallucination, test_member_variable_consistency | 2262 |
| sensor_process/task_002_camera_edge_detection | SUCCESS | 6/6 | None | 758 |
| sensor_process/task_003_rgbd_pointcloud_generation | FAILED | 4/6 | test_intrinsic_scaling_math, test_dispatch_logic_16uc1_32fc1 | 3927 |
| sensor_process/task_004_imu_odometry_estimation | FAILED | 3/6 | test_3d_rotation_coupling, test_acceleration_3d_projection, test_angular_singularity_mapping | 8294 |
| sensor_process/task_005_sonar_distance_estimation | FAILED | 1/6 | test_holonomic_kinematics, test_sonar_geometry_robust_v4, test_numerical_stability_epsilon, test_sonar_max_range_limit, test_sonar_y_mirroring_fix | 6219 |
| sensor_process/task_006_image_pipeline | FAILED | 7/9 | test_cv_bridge_exception_safety, test_template_dispatch_completeness | 3675 |
| sensor_process/task_007_hector_slam | FAILED | 6/7 | test_timestamp_preservation | 10341 |
| sensor_process/task_008_darknet | FAILED | 4/6 | test_timestamp_synchronization, test_move_semantics_on_publish | 10492 |
| sensor_process/task_009_lio_sam | FAILED | 5/6 | test_callback_group_initialization | 34148 |
| sensor_process/task_010_radar_velocity_estimation | FAILED | 5/8 | test_ros2_logging_migration, test_ros2_clock_usage, test_zero_timestamp_handling | 3098 |
| sensor_process/task_011_camera_lidar_fusion | FAILED | 3/6 | test_tf2_geometry_msgs_include, test_namespace_full_compliance, test_ukf_pipeline_integrity | 13098 |
