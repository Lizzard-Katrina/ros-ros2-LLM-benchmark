from setuptools import setup, find_packages

package_name = 'task_007_carla_ros_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='test',
    maintainer_email='test@test.com',
    description='CARLA ROS Bridge Migration Task',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'transforms_node = task_007_carla_ros_bridge.transforms_node:main',
        ],
    },
)