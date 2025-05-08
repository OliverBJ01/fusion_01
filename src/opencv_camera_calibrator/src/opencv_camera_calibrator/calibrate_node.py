#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
# ParameterDescriptor removed for Humble compatibility
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import yaml
import os
import time
import traceback # Added for better error reporting

class OpenCVCalibrator(Node):
    def __init__(self):
        super().__init__('opencv_camera_calibrator_node')

        # --- Parameters (Humble compatible declaration) ---
        self.declare_parameter('image_topic', '/usb_cam/image_raw')
        self.declare_parameter('camera_info_topic', '/usb_cam/camera_info')
        self.declare_parameter('chessboard_width', 8)
        self.declare_parameter('chessboard_height', 6)
        self.declare_parameter('square_size', 0.108) # MODIFY THIS TO YOUR MEASURED VALUE
        self.declare_parameter('min_captures', 15)
        self.declare_parameter('output_file', os.path.expanduser('~/.ros/camera_info/opencv_calibration.yaml'))
        self.declare_parameter('criteria_type', 'eps+max_iter')
        self.declare_parameter('criteria_max_iter', 30)
        self.declare_parameter('criteria_epsilon', 0.001)
        self.declare_parameter('calibration_flags', 'rational')

        # Get parameter values
        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.board_w = self.get_parameter('chessboard_width').value
        self.board_h = self.get_parameter('chessboard_height').value
        self.square_size = self.get_parameter('square_size').value
        self.min_captures = self.get_parameter('min_captures').value
        self.output_file = self.get_parameter('output_file').value

        # Reconstruct cornerSubPix criteria from parameters
        criteria_type_str = self.get_parameter('criteria_type').value.lower()
        crit_type = 0
        if 'eps' in criteria_type_str:
            crit_type |= cv2.TERM_CRITERIA_EPS
        if 'max_iter' in criteria_type_str:
            crit_type |= cv2.TERM_CRITERIA_MAX_ITER
        if crit_type == 0:
            self.get_logger().warn("Invalid criteria_type parameter, defaulting to EPS + MAX_ITER")
            crit_type = cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER
        self.criteria = (crit_type,
                         self.get_parameter('criteria_max_iter').value,
                         self.get_parameter('criteria_epsilon').value)

        # Reconstruct calibration flags
        calib_flags_str = self.get_parameter('calibration_flags').value.lower()
        self.calibration_flags = 0
        if 'rational' in calib_flags_str:
            self.calibration_flags |= cv2.CALIB_RATIONAL_MODEL
        elif 'fisheye' in calib_flags_str:
            self.calibration_flags |= cv2.CALIB_FISHEYE
            self.get_logger().warn("CALIB_FISHEYE flag set, but script uses standard calibrateCamera.")
        self.board_n = self.board_w * self.board_h
        self.board_dims = (self.board_w, self.board_h)

        self.get_logger().info(f"Chessboard size: {self.board_w}x{self.board_h}, Square size: {self.square_size}m")
        self.get_logger().info(f"Subscribing to image topic: {self.image_topic}")
        self.get_logger().info(f"Minimum captures needed: {self.min_captures}")
        self.get_logger().info(f"Output file: {self.output_file}")
        self.get_logger().info(f"CornerSubPix Criteria: {self.criteria}")
        self.get_logger().info(f"Calibration Flags: {self.calibration_flags}")

        # --- State Variables ---
        self.bridge = CvBridge()
        self.obj_points = []
        self.img_points = []
        self.last_image_time = self.get_clock().now()
        self.last_capture_time = self.get_clock().now()
        self.capture_delay = rclpy.duration.Duration(seconds=0.5)
        self.image_width = None
        self.image_height = None
        self.frame_id = "camera"
        self.cv_window_name = "Camera Calibration" # Define window name
        self.cv_window_created = False
        self.quit_requested = False
        # No need for _is_destroyed, rclpy handles internal state

        # Prepare object points
        self.objp = np.zeros((self.board_n, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.board_w, 0:self.board_h].T.reshape(-1, 2)
        self.objp *= self.square_size

        # --- ROS Subscriptions ---
        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.info_callback, 1)

        self.get_logger().info("OpenCV Camera Calibrator node started.")
        self.get_logger().info("Waiting for image dimensions from CameraInfo...")

    def info_callback(self, msg):
        if self.image_width is None:
            self.image_width = msg.width
            self.image_height = msg.height
            self.frame_id = msg.header.frame_id
            self.get_logger().info(f"Received image dimensions: {self.image_width}x{self.image_height}, frame_id: {self.frame_id}")
            if self.info_sub:
                self.destroy_subscription(self.info_sub)
                self.info_sub = None

    def image_callback(self, msg):
        if self.image_width is None or self.image_height is None or self.quit_requested:
            return

        now = self.get_clock().now()
        if (now - self.last_image_time) < rclpy.duration.Duration(seconds=0.05):
            return
        self.last_image_time = now

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        display_image = cv_image.copy()
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, self.board_dims, None)

        if ret:
            corners_subpix = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
            cv2.drawChessboardCorners(display_image, self.board_dims, corners_subpix, ret)

        # Add text overlay
        text_captures = f"Captures: {len(self.img_points)}/{self.min_captures}"
        text_controls = "Space: Capture | C: Calibrate | Q: Quit"
        cv2.putText(display_image, text_captures, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(display_image, text_controls, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # Ensure window is created before imshow
        if not self.cv_window_created:
            cv2.namedWindow(self.cv_window_name, cv2.WINDOW_AUTOSIZE)
            self.cv_window_created = True
        cv2.imshow(self.cv_window_name, display_image)

        # --- Handle Keyboard Input AFTER Displaying Image ---
        key_pressed = cv2.waitKey(1) & 0xFF

        if key_pressed == ord(' '):
            if ret and (now - self.last_capture_time) > self.capture_delay:
                self.last_capture_time = now
                self.img_points.append(corners_subpix)
                self.obj_points.append(self.objp)
                self.get_logger().info(f"Captured points: {len(self.img_points)}/{self.min_captures}")
            elif not ret:
                self.get_logger().warn("Space pressed, but no chessboard detected in this frame.")

        elif key_pressed == ord('c'):
            if len(self.img_points) >= self.min_captures:
                self.get_logger().info("Starting calibration...")
                self.calibrate_camera() # This sets quit_requested
                return # Exit callback
            else:
                self.get_logger().warn(f"Need at least {self.min_captures} captures. Have {len(self.img_points)}.")

        elif key_pressed == ord('q'):
            self.get_logger().info("Quit key pressed.")
            self.quit_requested = True # Signal main loop to exit
            return

        # Check if window was closed using the 'X' button
        if self.cv_window_created:
            try:
                if cv2.getWindowProperty(self.cv_window_name, cv2.WND_PROP_AUTOSIZE) < 0:
                    self.get_logger().info("OpenCV window closed manually.")
                    self.quit_requested = True
            except cv2.error:
                self.cv_window_created = False
                if not self.quit_requested:
                    self.get_logger().info("OpenCV window closed unexpectedly.")
                    self.quit_requested = True


    def calibrate_camera(self):
        if not self.obj_points or not self.img_points:
            self.get_logger().error("No points collected for calibration.")
            self.quit_requested = True
            return
        if self.image_width is None or self.image_height is None:
            self.get_logger().error("Image dimensions not available.")
            self.quit_requested = True
            return

        img_size = (self.image_width, self.image_height)
        self.get_logger().info(f"Calibrating with {len(self.obj_points)} views, size {img_size}...")

        try:
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                self.obj_points, self.img_points, img_size, None, None, flags=self.calibration_flags
            )
            if ret:
                self.get_logger().info("Calibration successful!")
                self.get_logger().info(f"Reprojection Error (RMS): {ret}")
                self.get_logger().info(f"Camera Matrix (K):\n{mtx}")
                self.get_logger().info(f"Distortion Coefficients (D):\n{dist}")
                mean_error = 0
                for i in range(len(self.obj_points)):
                    imgpoints2, _ = cv2.projectPoints(self.obj_points[i], rvecs[i], tvecs[i], mtx, dist)
                    error = cv2.norm(self.img_points[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
                    mean_error += error
                self.get_logger().info(f"Mean reprojection error (manual check): {mean_error / len(self.obj_points)}")
                self.save_calibration(mtx, dist)
            else:
                self.get_logger().error("Calibration failed!")
        except Exception as e:
            self.get_logger().error(f"Calibration error: {e}")
            traceback.print_exc()
        finally:
            # Ensure window is closed AFTER calibration attempt
            if self.cv_window_created:
                cv2.destroyAllWindows()
                self.cv_window_created = False
            self.quit_requested = True # Signal main loop to exit


    def save_calibration(self, camera_matrix, dist_coeffs):
        try:
            output_dir = os.path.dirname(self.output_file)
            if not os.path.exists(output_dir):
                os.makedirs(output_dir); self.get_logger().info(f"Created directory: {output_dir}")
            if self.calibration_flags & cv2.CALIB_RATIONAL_MODEL:
                distortion_model_name = 'rational_polynomial'; expected_coeffs = 8
            elif self.calibration_flags & cv2.CALIB_FISHEYE:
                distortion_model_name = 'fisheye'; expected_coeffs = 4
            else:
                distortion_model_name = 'plumb_bob'; expected_coeffs = 5

            dist_coeffs_list = dist_coeffs.flatten().tolist()
            if len(dist_coeffs_list) < expected_coeffs: dist_coeffs_list.extend([0.0]*(expected_coeffs-len(dist_coeffs_list)))
            elif len(dist_coeffs_list) > expected_coeffs: dist_coeffs_list = dist_coeffs_list[:expected_coeffs]

            # --- FIX: Convert projection matrix data to simple list ---
            proj_matrix_list = [
                camera_matrix[0,0], camera_matrix[0,1], camera_matrix[0,2], 0.0,
                camera_matrix[1,0], camera_matrix[1,1], camera_matrix[1,2], 0.0,
                camera_matrix[2,0], camera_matrix[2,1], camera_matrix[2,2], 0.0
            ]
            # --- End Fix ---

            calibration_data = {
                'image_width': self.image_width, 'image_height': self.image_height,
                'camera_name': os.path.splitext(os.path.basename(self.output_file))[0],
                'camera_matrix': {'rows': 3, 'cols': 3, 'data': camera_matrix.flatten().tolist()},
                'distortion_model': distortion_model_name,
                'distortion_coefficients': {'rows': 1, 'cols': len(dist_coeffs_list), 'data': dist_coeffs_list},
                'rectification_matrix': {'rows': 3, 'cols': 3, 'data': [1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]},
                'projection_matrix': {'rows': 3, 'cols': 4, 'data': proj_matrix_list } # Use the plain list
            }
            with open(self.output_file, 'w') as f: yaml.dump(calibration_data, f, default_flow_style=None, sort_keys=False)
            self.get_logger().info(f"Calibration saved to: {self.output_file}")
        except Exception as e: self.get_logger().error(f"Failed to save calibration: {e}"); traceback.print_exc()


    def run(self):
        """Runs the main loop."""
        self.get_logger().info("Entering main loop. Press keys in OpenCV window (Space, C, Q).")
        while rclpy.ok() and not self.quit_requested:
            if self.image_width is None or self.image_height is None:
                rclpy.spin_once(self, timeout_sec=0.1)
                continue

            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.01)

        self.get_logger().info("Exited main loop.")


def main(args=None):
    rclpy.init(args=args)
    calibrator_node = None
    exit_code = 0
    try:
        calibrator_node = OpenCVCalibrator()
        calibrator_node.run() # Run the loop
    except KeyboardInterrupt:
        logger = rclpy.logging.get_logger("opencv_camera_calibrator_main")
        logger.info("KeyboardInterrupt received.")
    except Exception as e:
        logger = rclpy.logging.get_logger("opencv_camera_calibrator_main")
        logger.fatal(f"Fatal error during execution: {e}")
        traceback.print_exc()
        exit_code = 1
    finally:
        # Ensure cleanup
        logger = rclpy.logging.get_logger("opencv_camera_calibrator_main")
        logger.info("Initiating final cleanup...")

        # Explicitly destroy window first
        cv2.destroyAllWindows()

        # --- FIX: Check if node exists and context is valid before destroying ---
        # Use standard ROS 2 Humble check for node validity if possible
        # A simple check if the variable exists and is not None is often sufficient
        if calibrator_node is not None:
             # No explicit is_destroyed() in Humble rclpy.node.Node
             # Rely on context check and destroy_node handling exceptions internally
             try:
                 # Check context before destroying
                 if rclpy.ok():
                     calibrator_node.destroy_node()
                     logger.info("Node destroyed.")
                 else:
                     logger.info("RCLPY context already invalid, skipping node destruction.")
             except Exception as e:
                 logger.warn(f"Error destroying node during cleanup: {e}")
        # --- End Fix ---

        if rclpy.ok():
            rclpy.shutdown()
            logger.info("RCLPY shutdown.")

        logger.info("Calibration script finished.")
        # sys.exit(exit_code) # Generally avoid sys.exit in ROS nodes


if __name__ == '__main__':
    main()


