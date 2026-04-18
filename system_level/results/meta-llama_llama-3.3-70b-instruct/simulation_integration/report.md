# System Level Benchmark Report: SIMULATION_INTEGRATION

| Task ID | Status | Score | Failed Functions | Tokens |
| :--- | :--- | :--- | :--- | :--- |
| simulation_integration/task_001_urdf | FAILED | 0/7 | test_urdf_base_link_consistency, test_urdf_inertial_integration, test_launch_xacro_command_api, test_launch_resource_indexing, test_launch_parameter_wrapping, test_anti_leakage_ros1_substitution, test_launch_integration_completeness | 334 |
| simulation_integration/task_002_gazebo_plugin | FAILED | 3/7 | test_launch_empty_world_inclusion, test_launch_world_parameter_passing, test_launch_xml_structure, test_anti_leakage_ros2_substitution | 885 |
| simulation_integration/task_003_turtlebot3_gazebo | FAILED | 1/9 | test_burger_base_link_mesh, test_burger_collision_geometry, test_burger_inertial_properties, test_waffle_dual_caster_structure, test_waffle_base_link_mesh, test_launch_uses_modern_gz_sim, test_launch_server_client_separation, test_no_legacy_gazebo_nodes | 1512 |
| simulation_integration/task_004_industrial_robot_simulator | FAILED | 2/4 | test_ros2_state_node_lifecycle, test_launch_system_migration_check | 6449 |
| simulation_integration/task_005_Allegro_Hand_V5_ROS1_Task | FAILED | 2/8 | test_cmake_ros2_ament_integrity, test_imu_physics_frequency_scaling_semantics, test_ros2_api_usage, test_msgbuffer_memory_logic, test_msgbuffer_safety_assertions, test_header_inclusion_semantics | 5734 |
| simulation_integration/task_006-ardupilot-gazebo | FAILED | 2/9 | test_socket_pollin_logic, test_socket_recv_semantics, test_preupdate_protocol_parsing, test_lockstep_logic, test_pwm_normalization_logic, test_joint_command_application, test_failsafe_handling | 323 |
| simulation_integration/task_007_carla_ros_bridge | FAILED | 3/8 | test_twist_linear_rotation_logic, test_twist_angular_unit_conversion, test_twist_angular_handedness_inversion, test_bridge_update_trigger_with_timestamp, test_actor_factory_pre_tick_update | 12429 |
| simulation_integration/task_008_navigation_recovery_action | FAILED | 1/4 | test_no_ros1_dependencies, test_subscriber_migration, test_service_client_and_call | 1530 |
