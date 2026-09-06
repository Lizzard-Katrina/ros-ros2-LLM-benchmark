from setuptools import setup, find_packages

package_name = 'task_005_KIE_multiagent_model'

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
    maintainer='test',
    maintainer_email='test@test.com',
    description='ROS2 migration of fkie_mas_discovery components',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'interface_finder_node = task_005_KIE_multiagent_model.interface_finder_node:main',
        ],
    },
)