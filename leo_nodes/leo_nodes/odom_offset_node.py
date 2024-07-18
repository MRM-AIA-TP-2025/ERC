#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import numpy as np

class OdomCorrectionNode(Node):
    def __init__(self):
        super().__init__('odom_offset_node')

        # Known Aruco tag poses in the world frame
        self.known_tag_poses = {
            1: np.array([-7.003807, 7.617856, 1.172268]),
            4: np.array([4.826193, 4.817856, 1.072268]),
            5: np.array([1.706193, -9.572144, 1.582268]),
            8: np.array([0.866193, 0.047856, 1.452268]),
            12: np.array([6.336193, -6.492144, 1.982268]),
            13: np.array([-8.823807, -11.012144, 1.792268])
        }

        # Placeholder for the latest odometry data
        self.current_odom = None

        # Placeholder for tvec data and tag ID
        self.tvec = None
        self.tag_id = None

        # Subscribe to /odom_unfiltered
        self.subscription = self.create_subscription(
            Odometry,
            '/odom_unfiltered',
            self.odom_callback,
            10)

        # Subscriber for aruco/tvec
        self.tvec_subscription = self.create_subscription(
            Float32MultiArray,
            'aruco/tvec',
            self.tvec_callback,
            10
        )

        # Publisher to /odom
        self.publisher = self.create_publisher(Odometry, '/odom', 10)

    def tvec_callback(self, msg):
        
        self.tag_id = int(msg.data[0])
        self.tvec = np.array(msg.data[1:])

    def odom_callback(self, msg):
        self.current_odom = msg
        
        if self.tvec is not None and self.tag_id in self.known_tag_poses:
            self.correct_odom(self.tvec, self.tag_id)
        else:
            # No Aruco detected or unknown tag ID, publish unfiltered odometry directly
            self.publisher.publish(self.current_odom)

    def correct_odom(self, tvec, tag_id):
        
        rover_pos = np.array([
            self.current_odom.pose.pose.position.x,
            self.current_odom.pose.pose.position.y,
            self.current_odom.pose.pose.position.z
        ])

       
        calculated_tag_pos = rover_pos + tvec

       
        known_tag_pose = self.known_tag_poses[tag_id]

       
        offset = known_tag_pose - calculated_tag_pos

        # Check if the offset magnitude is greater than 2
        if np.linalg.norm(offset) > 2:
            
            corrected_pos = rover_pos + offset

            
            corrected_odom = Odometry()
            corrected_odom.header = self.current_odom.header
            corrected_odom.child_frame_id = self.current_odom.child_frame_id
            corrected_odom.pose.pose.position.x = corrected_pos[0]
            corrected_odom.pose.pose.position.y = corrected_pos[1]
            corrected_odom.pose.pose.position.z = corrected_pos[2]
            corrected_odom.pose.pose.orientation = self.current_odom.pose.pose.orientation
            corrected_odom.pose.covariance = self.current_odom.pose.covariance
            corrected_odom.twist = self.current_odom.twist

            self.publisher.publish(corrected_odom)
        else:
            # Offset is not significant, publish unfiltered odometry directly
            self.publisher.publish(self.current_odom)

def main(args=None):
    rclpy.init(args=args)
    node = OdomCorrectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()