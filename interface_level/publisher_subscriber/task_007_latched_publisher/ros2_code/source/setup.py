from setuptools import setup

package_name = 'task_007_latched_publisher'

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
    description='ROS2 latched publisher using transient local QoS',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'latched_publisher = task_007_latched_publisher.latched_publisher:main',
        ],
    },
)