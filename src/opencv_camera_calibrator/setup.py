# ~/ros2_ws/src/opencv_camera_calibrator/setup.py

import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'opencv_camera_calibrator'

setup(
    name=package_name,
    version='0.0.0',
    # Still need find_packages for the Python package itself
    packages=find_packages(where='src', exclude=['test']),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='OpenCV based camera calibration node',
    license='Apache License 2.0',
    # Use 'scripts' pointing to the file within the 'src' layout
    scripts=[os.path.join('src', 'opencv_camera_calibrator', 'calibrate_node.py')],
    # Remove or comment out entry_points
    # entry_points={
    #     'console_scripts': [
    #         'calibrate_node = opencv_camera_calibrator.calibrate_node:main',
    #     ],
    # },
)

