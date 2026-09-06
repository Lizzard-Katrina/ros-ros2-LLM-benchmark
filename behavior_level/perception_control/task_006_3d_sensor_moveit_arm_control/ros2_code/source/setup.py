from setuptools import setup, find_packages

package_name = 'task_006_3d_sensor_moveit_arm_control'

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
    description='ROS2 Pick and Place Server',
    license='ISC',
    entry_points={
        'console_scripts': [
            'pick_and_place_server = task_006_3d_sensor_moveit_arm_control.pick_and_place_server:main',
        ],
    },
)