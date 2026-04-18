# Benchmark Report: SENSOR_PROCESS

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| sensor_process/task_001_laser_obstacle_detection | FAILED | 2/4 | test_filter_execution_flexible, test_member_variable_consistency | 2434 |
| sensor_process/task_002_camera_edge_detection | SUCCESS | 6/6 | None | 757 |
| sensor_process/task_003_rgbd_pointcloud_generation | FAILED | 3/6 | test_intrinsic_scaling_math, test_memory_unique_ownership, test_dispatch_logic_16uc1_32fc1 | 3830 |
| sensor_process/task_004_imu_odometry_estimation | FAILED | 2/6 | test_3d_rotation_coupling, test_acceleration_3d_projection, test_angular_singularity_mapping, test_state_member_coverage | 4924 |
| sensor_process/task_005_sonar_distance_estimation | FAILED | 2/6 | test_holonomic_kinematics, test_numerical_stability_epsilon, test_sonar_max_range_limit, test_sonar_y_mirroring_fix | 6582 |
| sensor_process/task_006_image_pipeline | FAILED | 5/9 | test_camera_info_scaling_logic, test_pointcloud2_field_setup, test_template_dispatch_completeness, test_radial_transform_update_check | 4294 |
| sensor_process/task_007_hector_slam | FAILED | 6/7 | test_timestamp_preservation | 10578 |
| sensor_process/task_008_darknet | FAILED | 3/6 | test_timestamp_synchronization, test_move_semantics_on_publish, test_coordinate_scaling_logic | 7573 |
| sensor_process/task_009_lio_sam | FAILED | 2/6 | test_callback_group_initialization, test_mutex_locking_in_service, test_service_response_success_set, test_no_legacy_ros1_symbols | 19986 |
| sensor_process/task_010_radar_velocity_estimation | FAILED | 5/8 | test_ros2_logging_migration, test_ros2_clock_usage, test_zero_timestamp_handling | 3696 |
| sensor_process/task_011_camera_lidar_fusion | FAILED | 3/6 | test_tf2_geometry_msgs_include, test_namespace_full_compliance, test_ukf_pipeline_integrity | 7110 |
