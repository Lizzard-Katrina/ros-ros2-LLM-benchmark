from setuptools import setup

package_name = 'task_002_sm_rosbag_recorder'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['recorder.cpp']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='ROS2 migration of rosbag recorder and talker',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'talker = task_002_sm_rosbag_recorder.talker_entry:main',
        ],
    },
)