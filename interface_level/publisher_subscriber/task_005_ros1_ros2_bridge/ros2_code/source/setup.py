from setuptools import setup

package_name = 'task_005_ros1_ros2_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='ROS1 to ROS2 Bridge Publisher/Listener',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'talker = task_005_ros1_ros2_bridge.talker:main',
            'listener = task_005_ros1_ros2_bridge.listener:main',
        ],
    },
)