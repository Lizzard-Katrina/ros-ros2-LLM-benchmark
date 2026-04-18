# Benchmark Report: SENSOR_PROCESS

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| sensor_process/task_001_laser_obstacle_detection | FAILED | 3/4 | test_member_variable_consistency | 5722 |
| sensor_process/task_002_camera_edge_detection | SUCCESS | 6/6 | None | 786 |
| sensor_process/task_003_rgbd_pointcloud_generation | SUCCESS | 6/6 | None | 7284 |
| sensor_process/task_004_imu_odometry_estimation | FAILED | 5/6 | test_acceleration_3d_projection | 10908 |
| sensor_process/task_005_sonar_distance_estimation | FAILED | 5/6 | test_sonar_max_range_limit | 8198 |
| sensor_process/task_006_image_pipeline | FAILED | 7/9 | test_cv_bridge_exception_safety, test_radial_transform_update_check | 8617 |
| sensor_process/task_007_hector_slam | SUCCESS | 7/7 | None | 14384 |
| sensor_process/task_008_darknet | FAILED | 5/6 | test_timestamp_synchronization | 13000 |
| sensor_process/task_009_lio_sam | FAILED | 5/6 | test_callback_group_initialization | 27763 |
| sensor_process/task_010_radar_velocity_estimation | FAILED | 5/8 | test_ros2_logging_migration, test_ros2_clock_usage, test_zero_timestamp_handling | 5962 |
| sensor_process/task_011_camera_lidar_fusion | FAILED | 4/6 | test_tf2_geometry_msgs_include, test_namespace_full_compliance | 15387 |
