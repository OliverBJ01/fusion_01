#lidar_launch.py

# Import necessary launch modules
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Generates the launch description for starting Velodyne nodes, TF, RViz, and subscriber."""

    # === Configuration Variables ===
    velodyne_ip = '10.1.1.201'
    velodyne_rpm = 300.0
    velodyne_model = 'VLP16'
    # Path to the calibration file (adjust if different from default install location)
    # Using get_package_share_directory ensures it finds the file within the ROS environment
    velodyne_pointcloud_pkg_dir = get_package_share_directory('velodyne_pointcloud')
    calibration_file = os.path.join(velodyne_pointcloud_pkg_dir, 'params', 'VLP16db.yaml')
    # Optional: Path to an RViz configuration file to load default settings
    # rviz_config_file = os.path.join(get_package_share_directory('your_launch_package_name'), 'rviz', 'velodyne_config.rviz') # Example path

    # === Node Definitions ===

    # 1. Velodyne Driver Node
    velodyne_driver = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        name='velodyne_driver_node', # Optional: Assign a specific node name
        output='screen', # Show node output directly in the terminal
        parameters=[{
            'device_ip': velodyne_ip,
            'rpm': velodyne_rpm,
            'model': velodyne_model,
            'frame_id': 'velodyne' # Explicitly set frame_id if needed
        }]
    )

    # 2. Velodyne Point Cloud Transformation Node
    velodyne_transformer = Node(
        package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        name='velodyne_transform_node',
        output='screen',
        parameters=[{
            'model': velodyne_model,
            'calibration': calibration_file,
            # Consider adding 'fixed_frame' and 'target_frame' if needed,
            # though defaults might work with the static transform publisher.
            # 'max_range': 100.0 # Example parameter
        }]
    )

    # 3. Static Transform Publisher (map -> velodyne)
    # Note: Arguments are passed differently than parameters
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_velodyne',
        output='screen',
        # Arguments format: x y z qx qy qz qw frame_id child_frame_id
        # Or: x y z yaw pitch roll frame_id child_frame_id (older format, quaternion preferred)
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'velodyne']
    )

    # 4. RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # Optional: Uncomment the line below to load a specific RViz configuration file
        # arguments=['-d', rviz_config_file]
    )

    # 5. Custom Lidar Subscriber Node
    lidar_subscriber = Node(
        package='lidar_subscriber_pkg', 
        executable='lidar_subscriber_node',
        name='lidar_subscriber_node',
        output='screen'
    )

    # === Launch Description Assembly ===
    # Create the LaunchDescription object and add all nodes to it
    ld = LaunchDescription()
    ld.add_action(velodyne_driver)
    ld.add_action(velodyne_transformer)
    ld.add_action(static_tf_publisher)
    ld.add_action(rviz_node)
    ld.add_action(lidar_subscriber)

    return ld

