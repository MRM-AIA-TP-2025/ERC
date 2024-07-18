#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('odom_compat_node')
        self.image_subscription = self.create_subscription(
            Image,
            'zed2_left_camera/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()

        # Camera calibration parameters
        self.camera_matrix = np.array([
            [529.965041, 0.0, 639.525559],
            [0.0, 529.940781, 360.667213],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.array([-0.001328, 0.000465, -0.000267, 0.000200, 0.0])
        
        # Update the dictionary and parameters fetching method
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_500)
        self.aruco_params = cv2.aruco.DetectorParameters()  # Use DetectorParameters
        self.aruco_size = 0.15  # ArUco marker size in meters

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            for i, corner in enumerate(corners):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, self.aruco_size, self.camera_matrix, self.dist_coeffs)
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                #cv2.aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
                self.get_logger().info(f"Detected ArUco marker ID: {ids[i][0]} at position {tvec[0][0]}")
        
        cv2.imshow('Aruco Detection', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()