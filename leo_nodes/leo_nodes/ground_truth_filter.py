#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros


class GroundTruthFilter(Node):
    def __init__(self):
        super().__init__('ground_truth_filter')

        self.x_offset = self.declare_parameter('x_offset', 0.0).value
        self.y_offset = self.declare_parameter('y_offset', 0.0).value
        self.z_offset = self.declare_parameter('z_offset', 0.0).value
        self.yaw_offset = self.declare_parameter('yaw_offset', 0.0).value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.ground_truth_pub = self.create_publisher(Odometry, 'ground_truth', 1)
        self.ground_truth_sub = self.create_subscription(
            Odometry, 'ground_truth_raw', self.ground_truth_raw_callback, 1
        )

    def ground_truth_raw_callback(self, msg: Odometry):
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_footprint',  # target frame
                msg.header.frame_id,  # source frame
                msg.header.stamp,  # time
                rclpy.time.Duration(seconds=1.0)  # timeout
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Failed to lookup transform: {str(e)}")
            return

        # Apply offsets to the pose
        transformed_pose = self.apply_offset(msg.pose.pose, transform.transform)

        # Publish the transformed odometry message
        msg.pose.pose = transformed_pose
        self.ground_truth_pub.publish(msg)

    def apply_offset(self, pose: Pose, transform: TransformStamped) -> Pose:
        # Convert transform to a translation vector and quaternion
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        # Apply offsets to the pose
        pose.position.x += self.x_offset - translation.x
        pose.position.y += self.y_offset - translation.y
        pose.position.z += self.z_offset - translation.z

        # Combine rotations (assuming both are quaternions)
        pose.orientation.x += self.yaw_offset - rotation.x
        pose.orientation.y += self.yaw_offset - rotation.y
        pose.orientation.z += self.yaw_offset - rotation.z
        pose.orientation.w += self.yaw_offset - rotation.w

        return pose


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
