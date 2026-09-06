from setuptools import setup

package_name = 'task_009_urdf'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['arm_urdf.urdf', 'manipulator.srdf', 'joint_limits.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='Manipulator URDF and MoveIt configuration',
    license='MIT',
)