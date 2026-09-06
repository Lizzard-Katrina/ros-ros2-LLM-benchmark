from setuptools import setup, find_packages

package_name = 'task_007_autonomous_navigation'

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
    description='Autonomous navigation patrol server',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_patrol_server = task_007_autonomous_navigation.turtle_patrol_server:main',
        ],
    },
)