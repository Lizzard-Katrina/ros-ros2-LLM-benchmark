from setuptools import setup, find_packages

package_name = 'task_006_flexbe_2'

setup(
    name=package_name,
    version='0.1.0',
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
    description='FlexBE Joint State Alignment - ROS2 migration',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'get_joint_values_state = task_006_flexbe_2.get_joint_values_state:main',
        ],
    },
)