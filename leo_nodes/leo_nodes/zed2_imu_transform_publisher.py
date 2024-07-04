#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import yaml
import argparse

class Zed2ImuTransformPublisher(Node):

    def __init__(self, yaml_file_path):
        super().__init__('zed2_imu_transform_publisher')

        with open(yaml_file_path, 'r') as file:
            params = yaml.safe_load(file)

        self._transform_msg = TransformStamped()
        self._transform_msg.header.frame_id = 'base_link'
        self._transform_msg.child_frame_id = 'camera_link'

        self._transform_msg.transform.translation.x = params['translation_x']
        self._transform_msg.transform.translation.y = params['translation_y']
        self._transform_msg.transform.translation.z = params['translation_z']

        self._transform_msg.transform.rotation.x = params['rotation_x']
        self._transform_msg.transform.rotation.y = params['rotation_y']
        self._transform_msg.transform.rotation.z = params['rotation_z']
        self._transform_msg.transform.rotation.w = params['rotation_w']

        self._publisher = self.create_publisher(TransformStamped, 'zed2/left_cam_imu_transform', 10)
        self._timer = self.create_timer(1.0, self.publish_transform)

    def publish_transform(self):
        self._transform_msg.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(self._transform_msg)
        self.get_logger().info('Publishing Zed2 IMU transform')

def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='Zed2 IMU Transform Publisher')
    parser.add_argument('yaml_file', type=str, help='Path to YAML parameter file')
    args = parser.parse_args()

    node = Zed2ImuTransformPublisher(args.yaml_file)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
