from setuptools import setup, find_packages

package_name = 'task_012_multimaster_flie'

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
    maintainer='developer',
    maintainer_email='dev@example.com',
    description='FKIE Multimaster Core Logic Migration to ROS2',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'master_monitor_node = task_012_multimaster_flie.master_monitor_node:main',
        ],
    },
)