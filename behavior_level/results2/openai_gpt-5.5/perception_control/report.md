# System Level Benchmark Report: PERCEPTION_CONTROL

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| perception_control/task_001_color_blob_tracking | FAILED | 4/7 | test_02_node_spin_and_image_subscription_pipeline, test_04_perception_scans_image_data_buffer, test_07_stop_and_motion_semantics_without_literal_numbers | 1556 |
| perception_control/task_002_camera_depth_reach_target | SUCCESS | 9/9 | None | 3312 |
| perception_control/task_003_multi_node_perception_control | FAILED | 2/5 | test_ds4_enum_mapping_present_and_used, test_l1_toggle_semantics_fidelity, test_axis_scaling_and_publish_gating_fidelity | 2063 |
| perception_control/task_004_mobile_lidar_obstacle_avoidance | FAILED | 7/9 | test_decision_uses_signed_cost_table_and_abs_cost, test_control_laws_and_modes_match_reference_semantics | 3090 |
| perception_control/task_005_ORBBEC | FAILED | 10/12 | test_frameset_forwards_non_color_streams_through_common_handler, test_color_thread_consumes_fifo_and_processes_in_pipeline_order | 49355 |
| perception_control/task_006_3d_sensor_moveit_arm_control | FAILED | 3/4 | test_4_async_fallback_mastery | 4786 |
| perception_control/task_007_autonomous_navigation | FAILED | 1/4 | test_turn_shortest_path_logic, test_proportional_control_law, test_emergency_safety_check | 3192 |
| perception_control/task_008_limo_robot | FAILED | 1/6 | test_ackermann_inverse_kinematics, test_steering_limit_clamping, test_odom_integration_frames, test_mecanum_lateral_awareness, test_time_differential_consistency | 10296 |
| perception_control/task_009_stretch_pipeline | FAILED | 3/6 | test_depth_unit_scaling, test_pinhole_projection_logic, test_ray_plane_analytical_solution | 8608 |
