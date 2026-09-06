from setuptools import setup, find_packages

package_name = 'task_002_ur5_pick_and_place'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@dev.com',
    description='UR5 Pick-and-Place migration from ROS1 to ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_planning = task_002_ur5_pick_and_place.motion_planning:main',
        ],
    },
)