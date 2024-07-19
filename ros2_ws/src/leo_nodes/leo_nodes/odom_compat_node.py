#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TwistWithCovariance
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from std_msgs.msg import Empty as EmptyMsg
from gazebo_msgs.srv import DeleteEntity


class OdomCompatNode(Node):
    def __init__(self):
        super().__init__('odom_compat_node')

        self.last_pose = Pose()
        self.last_pose.orientation.w = 1.0
        self.offset_matrix = None

        self.wheel_odom_pub = self.create_publisher(Odometry, 'wheel_odom_with_covariance', 1)
        self.wheel_odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.wheel_odom_callback,
            1
        )

        self.wheel_odom_reset_srv = self.create_service(
            Trigger,
            'firmware/reset_odometry',
            self.wheel_odom_reset
        )

        self.zed2_odom_pub = self.create_publisher(Odometry, 'zed2/odom', 1)
        self.zed2_odom_sub = self.create_subscription(
            Odometry,
            'gazebo/zed2/odom',
            self.zed2_odom_callback,
            1
        )

        self.zed2_reset_odom_client = self.create_client(DeleteEntity, 'gazebo/zed2/reset_odom')
        self.zed2_reset_odom_sub = self.create_subscription(
            EmptyMsg,
            'zed2/reset_odometry',
            self.zed2_reset_odom_callback,
            1
        )

        while not self.zed2_reset_odom_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        self.reset_zed2_odometry()

    def wheel_odom_callback(self, msg: Odometry):
        self.last_pose = msg.pose.pose

        if self.offset_matrix is not None:
            transformed_pose = self.apply_offset(msg.pose.pose)
            msg.pose.pose = transformed_pose

        self.wheel_odom_pub.publish(msg)

    def apply_offset(self, pose: Pose) -> Pose:
        # Apply offsets directly to pose (adjust as per your offsets)
        pose.position.x += 0.0  # Example offset adjustments
        pose.position.y += 0.0
        pose.position.z += 0.0

        return pose

    def wheel_odom_reset(self, req: Trigger.Request,):
        self.update_offset()
        return Trigger.Response(success=True, message="")

    def zed2_odom_callback(self, msg: Odometry):
        msg.twist = TwistWithCovariance()
        self.zed2_odom_pub.publish(msg)

    def zed2_reset_odom_callback(self, msg: EmptyMsg):
        self.reset_zed2_odometry()

    def reset_zed2_odometry(self):
        request = DeleteEntity.Request()
        request.name = "zed2"
        future = self.zed2_reset_odom_client.call_async(request)
        self.get_logger().info('Resetting ZED2 odometry')
        future.add_done_callback(self.reset_zed2_odom_done)

    def reset_zed2_odom_done(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('ZED2 odometry reset successfully')
            else:
                self.get_logger().error('Failed to reset ZED2 odometry')
        except Exception as e:
            self.get_logger().error(f'Exception while calling service: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = OdomCompatNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
