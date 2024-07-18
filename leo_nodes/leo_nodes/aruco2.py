#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import cv2

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('odom_compat_node')

        # Load camera calibration data
        calib_data_path = "/home/aditi/calib_data/MultiMatrix.npz"
        calib_data = np.load(calib_data_path)
        
        # Extract calibration parameters
        self.camera_matrix = calib_data["camMatrix"]
        self.dist_coeffs = calib_data["distCoef"]

        # ArUco marker parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_size = 0.027 # ArUco marker size in meters

        # Start the camera
        self.cap = cv2.VideoCapture(2)  # Change the index if needed

        self.process_camera()

    def process_camera(self):
        while self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

            if ids is not None:
                for i, corner in enumerate(corners):
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, self.aruco_size, self.camera_matrix, self.dist_coeffs)
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                    #cv2.aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
                    self.get_logger().info(f"Detected ArUco marker ID: {ids[i][0]} at position {tvec[0][0]}")

            cv2.imshow('Aruco Detection', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
