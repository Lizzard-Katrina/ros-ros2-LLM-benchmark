from setuptools import setup, find_packages

package_name = 'task_001_rosserial_python_integration'

setup(
    name=package_name,
    version='1.0.0',
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
    description='ROS 1 to ROS 2 migration of rosserial_python',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_node = task_001_rosserial_python_integration.serial_node:main',
        ],
    },
)