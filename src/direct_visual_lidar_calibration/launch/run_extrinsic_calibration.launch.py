# ~/ros2_ws/src/direct_visual_lidar_calibration/launch/run_extrinsic_calibration.launch.py

import os
import yaml   # For reading YAML file
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix # Use this to find the package install path
from launch.launch_context import LaunchContext # Needed for OpaqueFunction

# Define a function to read YAML and construct command arguments
# This function will be executed by OpaqueFunction during launch
def launch_setup(context: LaunchContext):
    # Get paths from launch configurations
    bag_parent_dir = LaunchConfiguration('bag_parent_dir').perform(context)
    preprocess_output_dir = LaunchConfiguration('preprocess_output_dir').perform(context)
    intrinsic_calib_yaml_path = LaunchConfiguration('intrinsic_calib_yaml_path').perform(context)

    # --- Construct full paths to executables using package prefix ---
    package_name = 'direct_visual_lidar_calibration'
    package_prefix_substitution = FindPackagePrefix(package_name)

    # Construct the path to the executable within the package's lib directory
    executable_preprocess_path = PathJoinSubstitution([
        package_prefix_substitution,
        'lib',
        package_name, # Executables are usually in install/<pkg>/lib/<pkg>/
        'preprocess'
    ])

    executable_initial_guess_manual_path = PathJoinSubstitution([
        package_prefix_substitution,
        'lib',
        package_name,
        'initial_guess_manual'
    ])

    executable_calibrate_path = PathJoinSubstitution([
        package_prefix_substitution,
        'lib',
        package_name,
        'calibrate'
    ])

    # --- Read Intrinsics from YAML ---
    camera_intrinsics_val = ""
    camera_distortion_coeffs_val = ""
    try:
        with open(intrinsic_calib_yaml_path, 'r') as file:
            calib_data = yaml.safe_load(file)
            if calib_data:
                # Extract Camera Matrix data (fx, fy, cx, cy)
                k_matrix = calib_data.get('camera_matrix', {}).get('data', [])
                if len(k_matrix) >= 6:
                    fx = k_matrix[0]
                    fy = k_matrix[4]
                    cx = k_matrix[2]
                    cy = k_matrix[5]
                    camera_intrinsics_val = f"{fx},{fy},{cx},{cy}" # Format without spaces
                else:
                    print(f"[WARN] Could not extract sufficient camera_matrix data from {intrinsic_calib_yaml_path}")

                # Extract Distortion Coefficients (k1, k2, p1, p2, k3)
                d_coeffs = calib_data.get('distortion_coefficients', {}).get('data', [])
                if len(d_coeffs) >= 5:
                    k1 = d_coeffs[0]
                    k2 = d_coeffs[1]
                    p1 = d_coeffs[2]
                    p2 = d_coeffs[3]
                    k3 = d_coeffs[4]
                    camera_distortion_coeffs_val = f"{k1},{k2},{p1},{p2},{k3}" # Format without spaces
                else:
                     print(f"[WARN] Could not extract sufficient distortion_coefficients data from {intrinsic_calib_yaml_path}")

            else:
                print(f"[ERROR] YAML file is empty or invalid: {intrinsic_calib_yaml_path}")

    except FileNotFoundError:
        print(f"[ERROR] Intrinsic calibration YAML file not found: {intrinsic_calib_yaml_path}")
    except yaml.YAMLError as e:
        print(f"[ERROR] Error parsing YAML file {intrinsic_calib_yaml_path}: {e}")
    except Exception as e:
         print(f"[ERROR] Error processing YAML file {intrinsic_calib_yaml_path}: {e}")

    # Raise error if parameters couldn't be read, as preprocess needs them
    if not camera_intrinsics_val or not camera_distortion_coeffs_val:
         raise RuntimeError("Failed to extract intrinsic/distortion parameters from YAML file. Cannot proceed.")



    # --- Other preprocess arguments ---
    camera_info_topic_val = '/usb_cam/camera_info'
    image_topic_val = '/usb_cam/image_raw'
    points_topic_val = '/velodyne_points'
    camera_model_val = 'plumb_bob'
    voxel_resolution_val = '0.2'

    # --- Define Processes using read values ---

    # Step 1: Preprocessing
    preprocess_cmd = [
        executable_preprocess_path, # Use the constructed path Substitution
        '--data_path', bag_parent_dir, # Use evaluated value
        '--dst_path', preprocess_output_dir, # Use evaluated value
        '--camera_info_topic', camera_info_topic_val,
        '--image_topic', image_topic_val,
        '--points_topic', points_topic_val,
        '--camera_model', camera_model_val,
        '--camera_intrinsics', camera_intrinsics_val, # Use value read from YAML
        '--camera_distortion_coeffs', camera_distortion_coeffs_val, # Use value read from YAML
        '--voxel_resolution', voxel_resolution_val,
        '--verbose',
    ]
    preprocess_process = ExecuteProcess(
        cmd=preprocess_cmd,
        output='screen',
    )

    # Step 2: Initial Guess
    initial_guess_cmd = [
        executable_initial_guess_manual_path, # Use the constructed path Substitution
        '--data_path', preprocess_output_dir, # Use evaluated value
    ]
    initial_guess_process = ExecuteProcess(
        cmd=initial_guess_cmd,
        output='screen',
    )

    # Step 3: Calibration
    calibrate_cmd = [
        executable_calibrate_path, # Use the constructed path Substitution
        '--data_path', preprocess_output_dir, # Use evaluated value
    ]
    calibrate_process = ExecuteProcess(
        cmd=calibrate_cmd,
        output='screen',
    )

    # Return the list of processes to launch for this context
    # Control which step runs by commenting/uncommenting lines HERE:
    return [
        #preprocess_process,
        initial_guess_process,
        # calibrate_process,
    ]


def generate_launch_description():
    # --- Declare Launch Arguments ---
    bag_parent_dir_arg = DeclareLaunchArgument(
        'bag_parent_dir',
        default_value=os.path.expanduser('~/ros2_ws/'),
        description='Parent directory of the ROS bag file(s) for calibration.'
    )

    preprocess_output_dir_arg = DeclareLaunchArgument(
        'preprocess_output_dir',
        default_value=os.path.join(os.getcwd(), 'preprocessed_calibration_data'),
        description='Directory to save preprocessed data and for calibrate node input.'
    )

    # --- NEW: Argument for the intrinsic calibration YAML file ---
    intrinsic_calib_yaml_path_arg = DeclareLaunchArgument(
        'intrinsic_calib_yaml_path',
        # Set default to the location of your latest calibration file
        default_value=os.path.expanduser('~/.ros/camera_info/arducam_8mm.yaml'),
        description='Path to the intrinsic camera calibration YAML file.'
    )

    # Use OpaqueFunction to allow reading the YAML file during launch execution
    opaque_function = OpaqueFunction(function=launch_setup)

    # --- LaunchDescription ---
    # Now includes the new argument and the OpaqueFunction
    ld = LaunchDescription([
        bag_parent_dir_arg,
        preprocess_output_dir_arg,
        intrinsic_calib_yaml_path_arg, # Add the new argument
        opaque_function # Add the OpaqueFunction which will define and return processes
    ])

    return ld


