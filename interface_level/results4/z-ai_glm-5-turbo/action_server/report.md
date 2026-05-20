# System Level Benchmark Report: ACTION_SERVER

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| action_server/task_001_base_case | FAILED | 6/9 | test_node_creation, test_execute_callback_exists, test_progress_loop_present | 968 |
| action_server/task_002_turtlebot3_patrol_action | FAILED | 4/7 | test_node_creation, test_action_server_creation, test_execute_callback_contains_patrol_logic | 2762 |
| action_server/task_003_pr2_gripper_action_server | FAILED | 7/9 | test_action_type_present, test_goalCB_exists | 5220 |
| action_server/task_004_turtle_bot_3 | FAILED | 9/12 | test_execute_callback_signature, test_execute_callback_no_return, test_feedback_published_iteratively | 989 |
| action_server/task_005_robot_voice_action | FAILED | 4/7 | test_control_logic_inside_timer_callback, test_ros_time_semantics_preserved, test_px4_qos_semantics_explicit | 3021 |
| action_server/task_006_amcl_navigation | FAILED | 3/10 | test_action_server_creation, test_handle_goal_defined, test_handle_cancel_defined, test_handle_accepted_defined, test_todo_comment_present, test_thread_usage_for_async, test_feedback_or_result_mentioned | 29471 |
| action_server/task_007_move_base_navigation_action | FAILED | 5/9 | test_feedback_publishing_exists, test_plan_swap_under_lock, test_recovery_behavior_loop, test_action_server_success_abort_calls | 20314 |
| action_server/task_008_follow_joint_trajectory_action | FAILED | 5/9 | test_class_exists, test_odom_lock_usage, test_warns_on_pose_fail, test_no_ros1_api_leftovers | 13493 |
| action_server/task_009_moveit_motion_planning_action | FAILED | 1/8 | test_class_exists, test_action_server_usage, test_rclcpp_node_exists, test_initialize_creates_server_with_callback, test_execute_callback_sets_result, test_preempt_callback_handles_cancel_and_flag, test_setMoveState_publishes_feedback_with_state | 4529 |
| action_server/task_010_pick_place | FAILED | 2/6 | test_move_group_interface_created, test_ros2_node_and_spinner, test_gripper_operations, test_post_place_open_gripper | 6360 |
| action_server/task_011_pr2_door | FAILED | 1/7 | test_door_action_client_created, test_move_base_action_client_created, test_wait_for_server_called, test_send_goal_and_wait_called, test_subscriber_exists, test_goal_initialization | 2254 |
| action_server/task_012_move_arm_cartesian_action | FAILED | 4/9 | test_ros2_node_initialization, test_move_group_interface_exists, test_path_constraints, test_cartesian_path_computation, test_collision_object_added | 10384 |
