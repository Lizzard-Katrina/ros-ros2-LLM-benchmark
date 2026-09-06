from setuptools import setup

package_name = 'task_001_simple_pub_sub'

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
    description='Simple Publisher and Subscriber (ROS2 Python)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'talker = task_001_simple_pub_sub.talker:main',
            'listener = task_001_simple_pub_sub.listener:main',
        ],
    },
)