from setuptools import setup, find_packages

package_name = 'task_004_simplified_pick_place'

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
    maintainer_email='dev@dev.com',
    description='Simplified Pick and Place - ROS2 migration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ez_pnp2 = task_004_simplified_pick_place.ez_pnp2:main',
            'test2_ez_pnp2 = task_004_simplified_pick_place.test2_ez_pnp2:main',
        ],
    },
)