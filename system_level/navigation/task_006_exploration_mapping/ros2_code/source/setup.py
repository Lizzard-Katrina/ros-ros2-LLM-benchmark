from setuptools import setup

package_name = 'task_006_exploration_mapping'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['mapping.launch', 'costmap_common_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='ROS1 to ROS2 translation of exploration mapping',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)