# launch_example.launch.py (optional ROS2 launch)
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package',
            executable='publisher_node',
            name='publisher_node',
            remappings=[('_____', 'remapped_topic')]
        ),
        Node(
            package='your_package',
            executable='subscriber_node',
            name='subscriber_node',
            remappings=[('_____', 'remapped_topic')]
        )
    ])
