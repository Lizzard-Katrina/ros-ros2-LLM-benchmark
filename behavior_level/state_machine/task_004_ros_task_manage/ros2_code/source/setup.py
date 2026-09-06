from setuptools import setup, find_packages

package_name = 'task_004_ros_task_manage'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='Task Manager SMACH wrapper migrated to ROS2',
    license='MIT',
    entry_points={
        'console_scripts': [
        ],
    },
)