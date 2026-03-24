# Benchmark Report: ACTION_SERVER

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| action_server/task_001_base_case | FAILED | 5/9 | test_no_ros1_artifacts, test_node_creation, test_execute_callback_exists, test_progress_loop_present | 834 |
| action_server/task_002_turtlebot3_patrol_action | FAILED | 4/7 | test_node_creation, test_action_server_creation, test_execute_callback_contains_patrol_logic | 2680 |
| action_server/task_003_pr2_gripper_action_server | FAILED | 8/9 | test_action_server_creation | 4040 |
| action_server/task_004_turtle_bot_3 | FAILED | 5/12 | test_execute_callback_signature, test_execute_callback_no_return, test_long_running_execution_present, test_feedback_object_created, test_feedback_published_iteratively, test_feedback_progress_updated, test_result_set_before_goal_termination | 616 |
| action_server/task_005_robot_voice_action | FAILED | 3/7 | test_control_logic_inside_timer_callback, test_px4_command_order_semantics, test_ros_time_semantics_preserved, test_px4_qos_semantics_explicit | 2392 |
| action_server/task_006_amcl_navigation | FAILED | 3/10 | test_action_server_creation, test_handle_goal_defined, test_handle_cancel_defined, test_handle_accepted_defined, test_todo_comment_present, test_thread_usage_for_async, test_feedback_or_result_mentioned | 27657 |
| action_server/task_007_move_base_navigation_action | FAILED | 0/9 | test_mutex_translation, test_feedback_publishing_exists, test_zero_velocity_publish_function, test_plan_swap_under_lock, test_velocity_command_computation, test_state_machine_structure, test_recovery_behavior_loop, test_ros2_logging, test_action_server_success_abort_calls | 26153 |
| action_server/task_008_follow_joint_trajectory_action | FAILED | 1/9 | test_class_exists, test_check_trajectory_method, test_get_robot_pose_used, test_update_map_logic, test_odom_lock_usage, test_tc_check_trajectory_args, test_returns_bool, test_warns_on_pose_fail | 22639 |
| action_server/task_009_moveit_motion_planning_action | FAILED | 2/8 | test_rclcpp_node_exists, test_initialize_creates_server_with_callback, test_execute_callback_sets_result, test_preempt_callback_handles_cancel_and_flag, test_setMoveState_publishes_feedback_with_state, test_no_ros1_artifacts | 2921 |
| action_server/task_010_pick_place | FAILED | 1/6 | test_move_group_interface_created, test_ros2_node_and_spinner, test_pick_place_sequence, test_gripper_operations, test_post_place_open_gripper | 4383 |
| action_server/task_011_pr2_door | FAILED | 3/7 | test_door_action_client_created, test_move_base_action_client_created, test_subscriber_exists, test_goal_initialization | 1996 |
| action_server/task_012_move_arm_cartesian_action | FAILED | 3/9 | test_ros2_node_initialization, test_move_group_interface_exists, test_path_constraints, test_cartesian_path_computation, test_collision_object_added, test_object_attached_to_robot | 9298 |
