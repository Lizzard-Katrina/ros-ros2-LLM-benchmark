# Benchmark Report: ACTION_SERVER

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| action_server/task_001_base_case | FAILED | 6/9 | test_node_creation, test_execute_callback_exists, test_progress_loop_present | 4044 |
| action_server/task_002_turtlebot3_patrol_action | FAILED | 4/7 | test_node_creation, test_action_server_creation, test_execute_callback_contains_patrol_logic | 7840 |
| action_server/task_003_pr2_gripper_action_server | FAILED | 8/9 | test_action_type_present | 16234 |
| action_server/task_004_turtle_bot_3 | FAILED | 6/12 | test_execute_callback_signature, test_execute_callback_no_return, test_long_running_execution_present, test_feedback_published_iteratively, test_feedback_progress_updated, test_result_set_before_goal_termination | 3479 |
| action_server/task_005_robot_voice_action | FAILED | 4/7 | test_control_logic_inside_timer_callback, test_ros_time_semantics_preserved, test_px4_qos_semantics_explicit | 5381 |
| action_server/task_006_amcl_navigation | TRANSLATION_FAILED | 0/0 | None | 0 |
| action_server/task_007_move_base_navigation_action | FAILED | 5/9 | test_mutex_translation, test_feedback_publishing_exists, test_plan_swap_under_lock, test_action_server_success_abort_calls | 40079 |
| action_server/task_008_follow_joint_trajectory_action | TRANSLATION_FAILED | 0/0 | None | 0 |
| action_server/task_009_moveit_motion_planning_action | FAILED | 5/8 | test_class_exists, test_rclcpp_node_exists, test_initialize_creates_server_with_callback | 22264 |
| action_server/task_010_pick_place | FAILED | 2/6 | test_move_group_interface_created, test_ros2_node_and_spinner, test_gripper_operations, test_post_place_open_gripper | 11123 |
| action_server/task_011_pr2_door | FAILED | 2/7 | test_door_action_client_created, test_move_base_action_client_created, test_wait_for_server_called, test_subscriber_exists, test_goal_initialization | 11543 |
| action_server/task_012_move_arm_cartesian_action | TRANSLATION_FAILED | 0/0 | None | 0 |
