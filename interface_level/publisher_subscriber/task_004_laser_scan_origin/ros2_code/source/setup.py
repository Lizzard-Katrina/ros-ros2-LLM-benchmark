from setuptools import setup

package_name = 'task_004_laser_scan_origin'

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
    description='LaserScan Basic Publisher/Subscriber - ROS2 migration',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'lidar_publisher = task_004_laser_scan_origin.lidar_publisher:main',
            'lidar_subscriber = task_004_laser_scan_origin.lidar_subscriber:main',
        ],
    },
)