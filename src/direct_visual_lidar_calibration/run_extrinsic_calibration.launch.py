# Example Configuration for direct_visual_lidar_calibration
# Save as e.g., ~/ros2_ws/src/direct_visual_lidar_calibration/config/my_extrinsic_calib.yaml

# --- General settings ---
# Path to the ROS bag file (can also be overridden in launch file)
# bag_filename: "/path/to/your/bjo_calibration_bag" # Example, often set in launch

# Frame IDs of your sensors
camera_frame_id: "usb_camera_link"  # The frame_id from your camera's CameraInfo
lidar_frame_id: "velodyne"         # The frame_id of your Velodyne LiDAR points

# --- Camera settings ---
camera_topic: "/usb_cam/image_raw"
camera_info_topic: "/usb_cam/camera_info"

# Camera model type: PINHOLE, FISHEYE, OMNIDIRECTIONAL
camera_model: "PINHOLE"

# --- LiDAR settings ---
lidar_topic: "/velodyne_points"

# LiDAR type: VELODYNE, OUSTER, HESAI, LIVVOX, PERFECT (for simulated perfect point cloud)
lidar_type: "VELODYNE"

# Voxel grid leaf size for downsampling input point clouds (meters)
# Adjust based on your LiDAR density and environment. Smaller values use more points.
voxel_resolution: 0.2

# --- Calibration settings ---
# Initial guess for extrinsic transformation (LiDAR to Camera)
# If you have a rough idea, provide it. Otherwise, it will try to estimate automatically.
# Format: [x, y, z, roll, pitch, yaw] (meters and radians)
# initial_guess_t_l_c: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # Example: no initial guess (or comment out)

# Number of frames to skip from the beginning of the bag
# Useful if the sensors are not stable at the start
skip_frames: 5

# Maximum number of frames to process from the bag
# Set to a large number or 0 to process all (after skipping)
max_frames: 0

# --- Preprocessing ---
# Dynamic point removal (removes points with high variance in range)
# enable_dynamic_removal: true
# dynamic_removal_num_scans: 5      # Number of scans to accumulate for variance check
# dynamic_removal_threshold_factor: 2.0 # Factor for thresholding

# --- Optimization settings ---
# Maximum number of ICP iterations
max_iterations: 50

# Maximum correspondence distance for ICP (meters)
# Points further than this will not be considered correspondences
max_correspondence_distance: 0.5

# Convergence criteria
# Stop if transformation difference between iterations is less than these
# transformation_epsilon: 1e-4
# rotation_epsilon: 1e-4

# --- Visualization ---
# Enable/disable visualization of point cloud projection during calibration
# enable_visualization: true

# --- Output ---
# Directory to save calibration results (e.g., transformation matrix, point cloud pairs)
# output_dir: "/path/to/your/calibration_results" # Often set in launch or defaults

