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
    