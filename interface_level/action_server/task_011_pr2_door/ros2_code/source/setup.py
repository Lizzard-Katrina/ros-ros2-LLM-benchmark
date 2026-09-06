from setuptools import setup, find_packages

package_name = 'task_011_pr2_door'

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
    description='PR2 Door Demo translated to ROS2',
    license='BSD',
    entry_points={
        'console_scripts': [
            'door_demo_test = task_011_pr2_door.door_demo_test_exec_test:main',
        ],
    },
)