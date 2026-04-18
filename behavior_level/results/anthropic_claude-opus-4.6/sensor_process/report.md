# Benchmark Report: SENSOR_PROCESS

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| sensor_process/task_001_laser_obstacle_detection | SUCCESS | 4/4 | None | 3282 |
| sensor_process/task_002_camera_edge_detection | SUCCESS | 6/6 | None | 908 |
| sensor_process/task_003_rgbd_pointcloud_generation | SUCCESS | 6/6 | None | 5645 |
| sensor_process/task_004_imu_odometry_estimation | FAILED | 4/6 | test_3d_rotation_coupling, test_acceleration_3d_projection | 12016 |
| sensor_process/task_005_sonar_distance_estimation | FAILED | 3/6 | test_holonomic_kinematics, test_sonar_max_range_limit, test_sonar_y_mirroring_fix | 9053 |
| sensor_process/task_006_image_pipeline | SUCCESS | 9/9 | None | 5677 |
| sensor_process/task_007_hector_slam | SUCCESS | 7/7 | None | 16387 |
| sensor_process/task_008_darknet | FAILED | 5/6 | test_flexible_logging_macros | 14387 |
| sensor_process/task_009_lio_sam | FAILED | 5/6 | test_callback_group_initialization | 50116 |
| sensor_process/task_010_radar_velocity_estimation | FAILED | 4/8 | test_ros2_logging_migration, test_ros2_clock_usage, test_mutex_lock_guard, test_zero_timestamp_handling | 4924 |
| sensor_process/task_011_camera_lidar_fusion | FAILED | 4/6 | test_tf2_geometry_msgs_include, test_ukf_pipeline_integrity | 19232 |
