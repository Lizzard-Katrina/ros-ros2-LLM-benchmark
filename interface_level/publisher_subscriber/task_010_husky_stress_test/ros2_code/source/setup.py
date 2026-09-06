from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'task_010_husky_stress_test'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['husky_empty_world.launch', 'husky.gazebo.xacro']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='Husky stress test - ROS2 translation',
    license='BSD',
    entry_points={
        'console_scripts': [
            'sensor_publisher_node = task_010_husky_stress_test.sensor_publisher_node:main',
        ],
    },
)