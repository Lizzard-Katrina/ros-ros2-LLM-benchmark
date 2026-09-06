from setuptools import setup, find_packages

package_name = 'task_006_pick_up_objects'

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
    description='ROS2 System-Level Integration: Manager-BT-Controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manage_objects_node = task_006_pick_up_objects.manage_objects_node:main',
            'pickup_behaviors_node = task_006_pick_up_objects.pickup_behaviors_node:main',
            'turtlebot_controller_node = task_006_pick_up_objects.turtlebot_controller_node:main',
        ],
    },
)