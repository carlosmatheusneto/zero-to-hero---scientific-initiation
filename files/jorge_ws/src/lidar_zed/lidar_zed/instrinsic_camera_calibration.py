import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from message_filters import Subscriber
from cv_bridge import CvBridge
import cv2
import yaml
import numpy as np
from datetime import datetime

from lidar_zed.read_yaml import extract_configuration

class CameraCalibration(Node):
    def __init__(self):
        super().init('camera_calibration')

        config = extract_configuration()

        self.image_topic = config['camera']['camera_topic']
        self.image_width = config['camera']['image_size']['width']
        self.image_length = config['camera']['image_size']['length']

        self.chessboard_rows = config['chessboard']['rows']
        self.chessboard_columns = config['chessboard']['columns']
        self.square_size = config['chessboard']['square_size_m']

        self.output_path = config['general']['config_folder']
        self.file = config['general']['camera_intrinsic_calibration']

        self.image_sub = Subscriber(self, Image, self.image_topic)
        self.bridge = CvBridge()

        self.obj_points = []
        self.img_points = []

        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TermCriteria_MAX_ITER, 30, 0.001)
        
        self.objp = np.zeros((self.chessboard_rows * self.chessboard_columns, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.chessboard_columns, 0:self.chessboard_rows].T.reshape(-1, 2)
        self.objp *= self.square_size

        self.get_logger().info("Camera calibration node initialized. Waiting for images...")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            ret, corners = cv2.findChessboardCorners(gray, (self.chessboard_columns, self.chessboard_rows), None)

            if ret:
                self.obj_points.append(self.objp)
                refined_corners = cv2.cornerSubPix(gray, corners, (11,11), (-1, -1), self.criteria)
                self.img_points.append(refined_corners)

                cv2.drawChessboardCorners(cv_image, (self.chessboard_columns, self.chessboard_rows), refined_corners, ret)
                self.get_logger().info("Chessboard detected and points added.")
            else:
                self.get_logger().warn("Chessboard not detected.")
            
            cv2.imshow("Image", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Failed to process the image: {e}")

    def save_calibration(self):
        if len(self.obj_points) < 10:
            self.get_logger().error("Not enough images for calibration. At least 10 are required.")
            return
    
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(self.obj_points, self.img_points, (self.image_width, self.image_length), None, None)

        calibrationdata = {
            'calibration_date': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'camera_matrix': {
                'rows' : 3,
                'columns' : 3,
                'data' :  camera_matrix.tolist()
            },
            'distortion_coefficients': {
                'rows': 1,
                'columns': len(dist_coeffs[0]),
                'data': dist_coeffs[0].tolist()
            },
            'image_size': {
                'width': 1920,
                'length': 1080,
            },
            'rsm_reprojection_error': ret
        }

        output_file = f"{self.output_path}/{self.file}"
        try:
            with open(output_file, 'w') as file:
                yaml.dump(calibrationdata, file)
            self.get_logger().info(f"calibration saved to {output_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to save the calibration: {e}")

def main(args = None):
    rclpy.init(args=args)
    node = CameraCalibration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_calibration()
        node.get_logger().info("Calibration process completed.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
