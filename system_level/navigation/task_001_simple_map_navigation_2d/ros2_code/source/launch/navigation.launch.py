import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('task_001_simple_map_navigation_2d')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml = LaunchConfiguration('map', default=os.path.join(pkg_share, 'maps', 'map.yaml'))
    params_file = LaunchConfiguration('params_file', default=os.path.join(pkg_share, 'config', 'nav2_params.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=map_yaml),
        DeclareLaunchArgument('params_file', default_value=params_file),
    ])