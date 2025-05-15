# ~/ros2_ws/src/my_calibration_launcher/launch/calibrate_camera.launch.py

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define the chessboard parameters
    # MODIFY THESE IF YOUR BOARD IS DIFFERENT
    chessboard_size = '10x7'
    chessboard_square_size = '0.025' # In meters

    # Define the topics from your camera node
    # MODIFY THESE IF YOUR TOPICS ARE DIFFERENT
    image_topic = '/usb_cam/image_raw'
    camera_info_topic = '/usb_cam/camera_info'

    # --- Camera Calibrator Node ---
    camera_calibrator_node = Node(
        package='camera_calibration',
        executable='cameracalibrator',
        # Pass chessboard parameters as arguments to the script
        arguments=[
            '--size', chessboard_size,
            '--square', chessboard_square_size
        ],
        # Use ROS 2 remapping rules
        remappings=[
            ('image', image_topic),
            ('camera_info', camera_info_topic)
            # Note: The node internally remaps 'camera' based on 'camera_info'
        ],
        output='screen',
        # Set parameters if needed, though usually not required for basic run
        # parameters=[
        #     {'param_name': 'value'}
        # ]
    )

    return LaunchDescription([
        camera_calibrator_node
    ])


