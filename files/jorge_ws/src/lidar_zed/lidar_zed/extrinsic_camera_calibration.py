import rclpy
from rclpy.node import Node
import yaml, os
import numpy as np
import cv2

from lidar_zed.read_yaml import extract_configuration


class get_extrinsic_calibration(Node):
    def __init__(self):
        super().__init__('get_extrinsic_calibration')

        config = extract_configuration()
        
        self.corr_file = config['general']['correspondence_file']
        self.corr_file = f'/ros2_ws/src/lidar_zed/data/{self.corr_file}'
        self.camera_yaml = config['general']['camera_intrinsic_calibration']
        self.camera_yaml = f'/ros2_ws/src/lidar_zed/config/{self.camera_yaml}'
        self.output_dir = config['general']['config_folder']
        self.output_file = config['general']['camera_extrinsic_calibration']

        self.get_logger().info('Starting extrinsic calibration...')
        self.solve_extrinsic_with_pnp()

    def open_camera_calibration(self, yaml_path: str):
        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)
        
        mat_data = config['camera_matrix']['data']
        camera_matrix = np.array(mat_data, np.float64)
        dist_data = config['distortion_coefficients']['data']
        dist_coeffs = np.array(dist_data, np.float64).reshape((-1,-1))

        return camera_matrix, dist_coeffs

    def solve_extrinsic_with_pnp(self):
        camera_matrix, dist_coeffs = self.open_camera_calibration(self.camera_yaml)
        
        self.get_logger().info(f"Camera matrix: \n {camera_matrix}")
        self.get_logger().info(f"Distortion coefficients: \n {dist_coeffs}")

        pts_2d = []
        pts_3d = []

        with open(self.corr_file, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                splitted = line.split(',')
                if len(splitted) != 5:
                    continue

                u, v, X, Y, Z = [float(val) for val in splitted]
                pts_2d.append([u, v])
                pts_3d.append([X, Y, Z])



def main(args=None):
    rclpy.init()
    node = get_extrinsic_calibration()
    try:
        rclpy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__init__':
    main()
