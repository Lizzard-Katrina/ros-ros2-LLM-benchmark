# Benchmark Report: SENSOR_PROCESS

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| sensor_process/task_001_laser_obstacle_detection | FAILED | 3/4 | test_member_variable_consistency | 8958 |
| sensor_process/task_002_camera_edge_detection | SUCCESS | 6/6 | None | 1566 |
| sensor_process/task_003_rgbd_pointcloud_generation | FAILED | 0/6 | test_intrinsic_scaling_math, test_offset_variable_usage, test_mandatory_kernel_call, test_header_and_frame_sync, test_memory_unique_ownership, test_dispatch_logic_16uc1_32fc1 | 3196 |
| sensor_process/task_004_imu_odometry_estimation | FAILED | 2/6 | test_3d_rotation_coupling, test_acceleration_3d_projection, test_angular_singularity_mapping, test_eigen_optimization | 13199 |
| sensor_process/task_005_sonar_distance_estimation | FAILED | 2/6 | test_holonomic_kinematics, test_numerical_stability_epsilon, test_sonar_max_range_limit, test_sonar_y_mirroring_fix | 11885 |
| sensor_process/task_006_image_pipeline | FAILED | 6/9 | test_pointcloud2_field_setup, test_template_dispatch_completeness, test_radial_transform_update_check | 10848 |
| sensor_process/task_007_hector_slam | SUCCESS | 7/7 | None | 13388 |
| sensor_process/task_008_darknet | FAILED | 5/6 | test_timestamp_synchronization | 12505 |
| sensor_process/task_009_lio_sam | FAILED | 5/6 | test_callback_group_initialization | 36602 |
| sensor_process/task_010_radar_velocity_estimation | FAILED | 4/8 | test_ros2_logging_migration, test_ros2_clock_usage, test_trigger_reset_logic_strict, test_zero_timestamp_handling | 6057 |
| sensor_process/task_011_camera_lidar_fusion | FAILED | 5/6 | test_tf2_geometry_msgs_include | 17304 |
