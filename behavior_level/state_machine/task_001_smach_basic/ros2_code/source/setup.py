from setuptools import setup, find_packages

package_name = 'task_001_smach_basic'

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
    maintainer_email='dev@example.com',
    description='SMACH ServiceState migration from ROS1 to ROS2',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service_state_node = task_001_smach_basic.service_state_node:main',
        ],
    },
)