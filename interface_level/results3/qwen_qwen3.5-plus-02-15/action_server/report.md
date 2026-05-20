# System Level Benchmark Report: ACTION_SERVER

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| action_server/task_001_base_case | FAILED | 7/9 | test_execute_callback_exists, test_progress_loop_present | 1206 |
| action_server/task_002_turtlebot3_patrol_action | FAILED | 4/7 | test_node_creation, test_action_server_creation, test_execute_callback_contains_patrol_logic | 3106 |
| action_server/task_003_pr2_gripper_action_server | FAILED | 8/9 | test_action_type_present | 5638 |
| action_server/task_004_turtle_bot_3 | FAILED | 7/12 | test_execute_callback_signature, test_execute_callback_no_return, test_feedback_published_iteratively, test_feedback_progress_updated, test_result_set_before_goal_termination | 950 |
| action_server/task_005_robot_voice_action | FAILED | 4/7 | test_control_logic_inside_timer_callback, test_ros_time_semantics_preserved, test_px4_qos_semantics_explicit | 3433 |
| action_server/task_006_amcl_navigation | FAILED | 3/10 | test_action_server_creation, test_handle_goal_defined, test_handle_cancel_defined, test_handle_accepted_defined, test_todo_comment_present, test_thread_usage_for_async, test_feedback_or_result_mentioned | 32551 |
| action_server/task_007_move_base_navigation_action | FAILED | 5/9 | test_mutex_translation, test_feedback_publishing_exists, test_plan_swap_under_lock, test_action_server_success_abort_calls | 21232 |
| action_server/task_008_follow_joint_trajectory_action | FAILED | 6/9 | test_class_exists, test_warns_on_pose_fail, test_no_ros1_api_leftovers | 14000 |
| action_server/task_009_moveit_motion_planning_action | FAILED | 4/8 | test_class_exists, test_action_server_usage, test_rclcpp_node_exists, test_initialize_creates_server_with_callback | 5175 |
| action_server/task_010_pick_place | FAILED | 3/6 | test_move_group_interface_created, test_ros2_node_and_spinner, test_post_place_open_gripper | 5944 |
| action_server/task_011_pr2_door | FAILED | 2/7 | test_door_action_client_created, test_move_base_action_client_created, test_wait_for_server_called, test_subscriber_exists, test_goal_initialization | 2748 |
| action_server/task_012_move_arm_cartesian_action | FAILED | 4/9 | test_ros2_node_initialization, test_move_group_interface_exists, test_path_constraints, test_cartesian_path_computation, test_collision_object_added | 11208 |
