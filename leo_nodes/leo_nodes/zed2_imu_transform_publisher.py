#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from camera_calibration_parsers import readCalibration

class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('zed2_imu_transform_publisher')
        self.declare_parameter('camera_name', 'zed2_left_optical_camera')
        self.declare_parameter('yaml_file', '/home/aditi/ERC/leo_rover/params/zed2_left_camera_calibration.yaml')

        camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        yaml_file = self.get_parameter('yaml_file').get_parameter_value().string_value

        camera_info_msg = CameraInfo()
        calibration_data = readCalibration(yaml_file, camera_name)
        if calibration_data:
            camera_info_msg.k = calibration_data[1]['camera_matrix']['data']
            camera_info_msg.d = calibration_data[1]['distortion_coefficients']['data']
            camera_info_msg.r = calibration_data[1]['rectification_matrix']['data']
            camera_info_msg.p = calibration_data[1]['projection_matrix']['data']
            camera_info_msg.width = calibration_data[1]['image_width']
            camera_info_msg.height = calibration_data[1]['image_height']
        else:
            self.get_logger().error('Failed to read calibration data from file')

        self.publisher_ = self.create_publisher(CameraInfo, 'camera_info', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, lambda: self.publisher_.publish(camera_info_msg))

def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
