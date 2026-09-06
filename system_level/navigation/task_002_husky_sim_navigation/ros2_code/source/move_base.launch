import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directories
    pkg_share = get_package_share_directory('task_002_husky_sim_navigation')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    # Paths to config and map files
    default_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_map_file = os.path.join(pkg_share, 'maps', 'empty_map.yaml')

    # Declare launch arguments
    no_static_map_arg = DeclareLaunchArgument(
        'no_static_map',
        default_value='false',
        description='If true, do not use a static map (use SLAM instead).'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true.'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the Nav2 parameters file.'
    )

    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Full path to the map yaml file to load.'
    )

    # Launch configurations
    no_static_map = LaunchConfiguration('no_static_map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')

    # --- Static map mode (no_static_map == false) ---
    # Use Nav2 with a pre-built map via map_server + AMCL localization
    static_map_group = GroupAction(
        condition=UnlessCondition(no_static_map),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'map': map_yaml_file,
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                }.items(),
            ),
        ],
    )

    # --- SLAM mode (no_static_map == true) ---
    # Use slam_toolbox for online SLAM instead of a static map
    slam_group = GroupAction(
        condition=IfCondition(no_static_map),
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    params_file,
                    {'use_sim_time': use_sim_time},
                ],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_slam',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': [
                        'slam_toolbox',
                    ]},
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                }.items(),
            ),
        ],
    )

    return LaunchDescription([
        no_static_map_arg,
        use_sim_time_arg,
        params_file_arg,
        map_file_arg,
        static_map_group,
        slam_group,
    ])