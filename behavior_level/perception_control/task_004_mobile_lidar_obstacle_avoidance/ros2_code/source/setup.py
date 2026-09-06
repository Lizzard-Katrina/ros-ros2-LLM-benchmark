from setuptools import setup

package_name = 'task_004_mobile_lidar_obstacle_avoidance'

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
    maintainer_email='dev@dev.com',
    description='Mobile robot obstacle avoidance using LiDAR - ROS2 translation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'laser_obstacle_avoid_360_node = task_004_mobile_lidar_obstacle_avoidance.laser_obstacle_avoid_360_node:main',
        ],
    },
)