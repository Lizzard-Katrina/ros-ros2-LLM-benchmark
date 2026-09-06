from setuptools import setup, find_packages

package_name = 'task_006_dynamic_param_rqt_reconfigure'

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
    maintainer='test',
    maintainer_email='test@test.com',
    description='Dynamic parameter async-sync bridge',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'param_server_node = task_006_dynamic_param_rqt_reconfigure.param_server_node:main',
        ],
    },
)