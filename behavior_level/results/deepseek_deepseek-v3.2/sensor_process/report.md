# Benchmark Report: SENSOR_PROCESS

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| sensor_process/task_001_laser_obstacle_detection | FAILED | 3/4 | test_member_variable_consistency | 2673 |
| sensor_process/task_002_camera_edge_detection | SUCCESS | 6/6 | None | 714 |
| sensor_process/task_003_rgbd_pointcloud_generation | FAILED | 5/6 | test_intrinsic_scaling_math | 5144 |
| sensor_process/task_004_imu_odometry_estimation | FAILED | 2/6 | test_3d_rotation_coupling, test_acceleration_3d_projection, test_angular_singularity_mapping, test_state_member_coverage | 9935 |
| sensor_process/task_005_sonar_distance_estimation | FAILED | 1/6 | test_holonomic_kinematics, test_sonar_geometry_robust_v4, test_numerical_stability_epsilon, test_sonar_max_range_limit, test_sonar_y_mirroring_fix | 7680 |
| sensor_process/task_006_image_pipeline | FAILED | 6/9 | test_pointcloud2_field_setup, test_template_dispatch_completeness, test_radial_transform_update_check | 4866 |
| sensor_process/task_007_hector_slam | SUCCESS | 7/7 | None | 12107 |
| sensor_process/task_008_darknet | FAILED | 5/6 | test_timestamp_synchronization | 11867 |
| sensor_process/task_009_lio_sam | FAILED | 5/6 | test_callback_group_initialization | 27467 |
| sensor_process/task_010_radar_velocity_estimation | FAILED | 5/8 | test_ros2_logging_migration, test_ros2_clock_usage, test_zero_timestamp_handling | 4184 |
| sensor_process/task_011_camera_lidar_fusion | FAILED | 4/6 | test_tf2_geometry_msgs_include, test_namespace_full_compliance | 15490 |
