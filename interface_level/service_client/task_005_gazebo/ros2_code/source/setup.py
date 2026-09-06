from setuptools import setup, find_packages

package_name = 'task_005_gazebo'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='Gazebo ROS2 service client wrappers',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gazebo_interface_node = task_005_gazebo.gazebo_interface_node:main',
        ],
    },
)