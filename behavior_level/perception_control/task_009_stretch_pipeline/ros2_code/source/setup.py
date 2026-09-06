from setuptools import setup, find_packages

package_name = 'task_009_stretch_pipeline'

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
    description='Stretch 3D Perception Pipeline',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_node = task_009_stretch_pipeline.detection_2d_to_3d:main',
        ],
    },
)