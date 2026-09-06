import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('urdf_tutorial')
    default_model_path = os.path.join(pkg_share, 'urdf', '08-macroed.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro', ' ', default_model_path]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(
            Command(['xacro', ' ', default_model_path]),
            value_type=str
        )}]
    )

    urdf_launch_share = get_package_share_directory('urdf_launch')

    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(urdf_launch_share, 'launch', 'display.launch.py')
        ),
        launch_arguments={'urdf_package': 'urdf_tutorial',
                          'urdf_package_path': os.path.join('urdf', '08-macroed.urdf.xacro')}.items()
    )

    return LaunchDescription([
        display_launch,
    ])