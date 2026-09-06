from setuptools import setup, find_packages

package_name = 'task_005_interbotix_demo'

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
    description='ROS2 migration of Interbotix XSArm perception pick and place demo',
    license='BSD',
    entry_points={
        'console_scripts': [
            'pick_and_place = task_005_interbotix_demo.pick_and_place:main',
        ],
    },
)