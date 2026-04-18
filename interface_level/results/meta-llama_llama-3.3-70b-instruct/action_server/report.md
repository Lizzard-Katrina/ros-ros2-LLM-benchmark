# Benchmark Report: ACTION_SERVER

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| action_server/task_001_base_case | FAILED | 6/9 | test_node_creation, test_execute_callback_exists, test_progress_loop_present | 788 |
| action_server/task_002_turtlebot3_patrol_action | FAILED | 4/7 | test_node_creation, test_action_server_creation, test_execute_callback_contains_patrol_logic | 2707 |
| action_server/task_003_pr2_gripper_action_server | FAILED | 7/9 | test_action_type_present, test_action_server_creation | 4346 |
| action_server/task_004_turtle_bot_3 | FAILED | 7/12 | test_execute_callback_signature, test_execute_callback_no_return, test_feedback_published_iteratively, test_feedback_progress_updated, test_result_set_before_goal_termination | 724 |
| action_server/task_005_robot_voice_action | FAILED | 1/7 | test_ros1_control_loop_migrated_to_ros2_timer, test_control_logic_inside_timer_callback, test_offboard_heartbeat_semantics_preserved, test_px4_command_order_semantics, test_ros_time_semantics_preserved, test_px4_qos_semantics_explicit | 1824 |
| action_server/task_006_amcl_navigation | FAILED | 2/10 | test_action_server_creation, test_handle_goal_defined, test_handle_cancel_defined, test_handle_accepted_defined, test_todo_comment_present, test_laser_data_handling_mentioned, test_thread_usage_for_async, test_feedback_or_result_mentioned | 15674 |
| action_server/task_007_move_base_navigation_action | FAILED | 4/9 | test_feedback_publishing_exists, test_plan_swap_under_lock, test_velocity_command_computation, test_recovery_behavior_loop, test_action_server_success_abort_calls | 19989 |
| action_server/task_008_follow_joint_trajectory_action | FAILED | 6/9 | test_class_exists, test_warns_on_pose_fail, test_no_ros1_api_leftovers | 12812 |
| action_server/task_009_moveit_motion_planning_action | FAILED | 2/8 | test_class_exists, test_action_server_usage, test_initialize_creates_server_with_callback, test_execute_callback_sets_result, test_preempt_callback_handles_cancel_and_flag, test_setMoveState_publishes_feedback_with_state | 4292 |
| action_server/task_010_pick_place | FAILED | 1/6 | test_move_group_interface_created, test_ros2_node_and_spinner, test_pick_place_sequence, test_gripper_operations, test_post_place_open_gripper | 4771 |
| action_server/task_011_pr2_door | FAILED | 3/7 | test_door_action_client_created, test_move_base_action_client_created, test_subscriber_exists, test_goal_initialization | 2029 |
| action_server/task_012_move_arm_cartesian_action | FAILED | 4/9 | test_ros2_node_initialization, test_move_group_interface_exists, test_path_constraints, test_cartesian_path_computation, test_collision_object_added | 9688 |
