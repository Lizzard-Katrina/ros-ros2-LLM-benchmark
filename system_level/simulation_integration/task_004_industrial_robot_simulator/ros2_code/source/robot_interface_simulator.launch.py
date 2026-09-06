from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare the controller_joint_names parameter for the 6-DOF manipulator
    controller_joint_names_arg = DeclareLaunchArgument(
        'controller_joint_names',
        default_value="['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']",
        description='List of joint names for the 6-DOF industrial robot'
    )

    # Robot state node
    robot_state_node = Node(
        package='industrial_robot_client',
        executable='robot_state',
        name='robot_state',
        output='screen',
        parameters=[{
            'controller_joint_names': [
                'joint_1', 'joint_2', 'joint_3',
                'joint_4', 'joint_5', 'joint_6'
            ]
        }]
    )

    # Joint trajectory action node
    joint_trajectory_action_node = Node(
        package='industrial_robot_client',
        executable='joint_trajectory_action',
        name='joint_trajectory_action',
        output='screen',
        parameters=[{
            'controller_joint_names': [
                'joint_1', 'joint_2', 'joint_3',
                'joint_4', 'joint_5', 'joint_6'
            ]
        }]
    )

    return LaunchDescription([
        controller_joint_names_arg,
        robot_state_node,
        joint_trajectory_action_node,
    ])