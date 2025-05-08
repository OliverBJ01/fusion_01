# ~/ros2_ws/src/my_calibration_launcher/setup.py

import os
from setuptools import find_packages, setup

package_name = 'my_calibration_launcher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Only install package marker and package.xml for now
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
            [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Launch files for camera calibration',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
        ],
    },
)

