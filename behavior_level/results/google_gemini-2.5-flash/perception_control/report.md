# Benchmark Report: PERCEPTION_CONTROL

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| perception_control/task_001_color_blob_tracking | FAILED | 5/7 | test_02_node_spin_and_image_subscription_pipeline, test_07_stop_and_motion_semantics_without_literal_numbers | 1660 |
| perception_control/task_002_camera_depth_reach_target | SUCCESS | 9/9 | None | 3704 |
| perception_control/task_003_multi_node_perception_control | FAILED | 2/5 | test_ds4_enum_mapping_present_and_used, test_l1_toggle_semantics_fidelity, test_axis_scaling_and_publish_gating_fidelity | 2326 |
| perception_control/task_004_mobile_lidar_obstacle_avoidance | FAILED | 6/9 | test_partitions_ranges_by_slicing, test_decision_uses_signed_cost_table_and_abs_cost, test_control_laws_and_modes_match_reference_semantics | 4772 |
| perception_control/task_005_ORBBEC | FAILED | 7/12 | test_frameset_alignment_semantics_present, test_frameset_forwards_non_color_streams_through_common_handler, test_color_thread_consumes_fifo_and_processes_in_pipeline_order, test_single_frame_has_subscriber_gate_includes_image_and_camerainfo, test_single_frame_supports_flip_branch_and_depth_scaling_hook | 68151 |
| perception_control/task_006_3d_sensor_moveit_arm_control | FAILED | 2/4 | test_2_wait_logic_atomic, test_4_async_fallback_mastery | 6131 |
| perception_control/task_007_autonomous_navigation | FAILED | 2/4 | test_proportional_control_law, test_emergency_safety_check | 3583 |
| perception_control/task_008_limo_robot | FAILED | 1/6 | test_ackermann_inverse_kinematics, test_steering_limit_clamping, test_odom_integration_frames, test_mecanum_lateral_awareness, test_time_differential_consistency | 12713 |
| perception_control/task_009_stretch_pipeline | SUCCESS | 6/6 | None | 10789 |
