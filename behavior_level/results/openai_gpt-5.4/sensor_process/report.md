# Benchmark Report: SENSOR_PROCESS

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| sensor_process/task_001_laser_obstacle_detection | FAILED | 3/4 | test_member_variable_consistency | 2602 |
| sensor_process/task_002_camera_edge_detection | SUCCESS | 6/6 | None | 646 |
| sensor_process/task_003_rgbd_pointcloud_generation | FAILED | 5/6 | test_intrinsic_scaling_math | 4359 |
| sensor_process/task_004_imu_odometry_estimation | FAILED | 4/6 | test_3d_rotation_coupling, test_acceleration_3d_projection | 8673 |
| sensor_process/task_005_sonar_distance_estimation | FAILED | 4/6 | test_holonomic_kinematics, test_sonar_max_range_limit | 6542 |
| sensor_process/task_006_image_pipeline | FAILED | 7/9 | test_cv_bridge_exception_safety, test_radial_transform_update_check | 4494 |
| sensor_process/task_007_hector_slam | SUCCESS | 7/7 | None | 11128 |
| sensor_process/task_008_darknet | FAILED | 3/6 | test_flexible_logging_macros, test_flexible_mutex_locking, test_timestamp_synchronization | 10994 |
| sensor_process/task_009_lio_sam | FAILED | 5/6 | test_callback_group_initialization | 35702 |
| sensor_process/task_010_radar_velocity_estimation | FAILED | 4/8 | test_ros2_logging_migration, test_ros2_clock_usage, test_mutex_lock_guard, test_zero_timestamp_handling | 3709 |
| sensor_process/task_011_camera_lidar_fusion | FAILED | 4/6 | test_tf2_geometry_msgs_include, test_namespace_full_compliance | 13816 |
