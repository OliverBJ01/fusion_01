# camera_lidar_launch.py (Revised Include Method)

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable
# Import necessary actions for including other launch files
from launch.actions import IncludeLaunchDescription
# PythonLaunchDescriptionSource import no longer strictly needed but fine to keep
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # --- USB Camera Configuration ---
    # Define the package name where the config file is located
    usb_cam_package_name = 'usb_cam'
    # Define the relative path to the config file within that package's share directory
    usb_cam_config_relative_path = os.path.join('config', 'arducam_params.yaml')

    # Construct the full path to the YAML parameter file
    usb_cam_params_file = os.path.join(
        get_package_share_directory(usb_cam_package_name),
        usb_cam_config_relative_path
    )

    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='v4l2_usb_cam',
        parameters=[usb_cam_params_file],
        remappings=[
            ('image_raw', '/usb_cam/image_raw'),
            ('camera_info', '/usb_cam/camera_info'),
        ],
        output='screen',
    )


    # --- Include LiDAR Launch File ---
    lidar_package_name = 'lidar_subscriber_pkg'
    lidar_launch_file_name = 'lidar_launch.py'

    lidar_launch_file_path = os.path.join(
        get_package_share_directory(lidar_package_name),
        'launch', # Assuming the launch file is in a 'launch' subdirectory
        lidar_launch_file_name
    )

    # *** Try passing the path string directly ***
    lidar_launch = IncludeLaunchDescription(
        lidar_launch_file_path, # Pass the string path instead of the Source object
        # Add launch_arguments={'arg_name': 'value'}.items() here if your
        # included launch file requires arguments to be passed in.
    )

    # --- Assemble Launch Description ---
    ld = LaunchDescription()
    ld.add_action(usb_cam_node)
    # ld.add_action(gscam_node) # GSCAM node was removed in user's last code block
    ld.add_action(lidar_launch)

    return ld
