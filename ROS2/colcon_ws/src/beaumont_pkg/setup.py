from setuptools import find_packages, setup
import os
from glob import glob 

package_name = 'beaumont_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'i2c_master = beaumont_pkg.i2c_master:main',
            'task_manager = beaumont_pkg.task_manager:main',
            'profiles_manager = beaumont_pkg.profiles_manager:main',
            'simulation_thruster_control = beaumont_pkg.simulation_thruster_control:main',
            'simulation_camera_subscriber = beaumont_pkg.simulation_camera_subscriber:main',
            'simulation_tooling_control = beaumont_pkg.simulation_tooling_control:main',
            'simulation_autonomous_movement = beaumont_pkg.simulation_autonomous_movement:main',
        ],
    },
)
