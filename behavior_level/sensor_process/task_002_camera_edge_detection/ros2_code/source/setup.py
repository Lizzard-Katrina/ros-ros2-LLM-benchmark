from setuptools import setup

package_name = 'task_002_camera_edge_detection'

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
    description='Camera Edge Detection Node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'edge_detection_node = task_002_camera_edge_detection.camera_edge:main',
        ],
    },
)