#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import tf2_ros
from geometry_msgs.msg import Transform, TransformStamped


class Zed2ImuTransformBroadcaster(Node):
    def __init__(self):
        super().__init__('zed2_imu_transform_broadcaster')

        self.tf_frame_prefix = self.declare_parameter('tf_frame_prefix', '').value

        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        self.create_subscription(
            Transform,
            'zed2/left_cam_imu_transform',
            self.imu_transform_callback,
            1
        )

    def imu_transform_callback(self, msg: Transform):
        imu_transform = TransformStamped()

        imu_transform.header.stamp = self.get_clock().now().to_msg()
        imu_transform.header.frame_id = self.tf_frame_prefix + 'zed2_left_camera_frame'
        imu_transform.child_frame_id = self.tf_frame_prefix + 'zed2_imu_link'

        imu_transform.transform = msg

        self.broadcaster.sendTransform(imu_transform)


def main(args=None):
    rclpy.init(args=args)
    node = Zed2ImuTransformBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
