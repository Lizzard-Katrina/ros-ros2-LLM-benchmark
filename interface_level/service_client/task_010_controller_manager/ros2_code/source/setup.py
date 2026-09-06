from setuptools import setup, find_packages

package_name = 'task_010_controller_manager'

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
    description='ROS2 migration of controller_manager_interface',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_manager_interface = task_010_controller_manager.controller_manager_interface:main',
        ],
    },
)