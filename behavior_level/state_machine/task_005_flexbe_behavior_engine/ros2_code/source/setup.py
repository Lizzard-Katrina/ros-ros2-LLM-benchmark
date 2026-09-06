from setuptools import setup, find_packages

package_name = 'task_005_flexbe_behavior_engine'

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
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='FlexBE MoveBaseState migrated to Nav2',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'move_base_state = task_005_flexbe_behavior_engine.move_base_state:main',
        ],
    },
)