#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import pcl
from pcl import PointCloud_PointXYZ
from sensor_msgs_py import point_cloud2
import numpy as np

class PCL(Node):
    def __init__(self):
        super().__init__('pcl_node')
        self.filtered_pcl_publisher = self.create_publisher(PointCloud2, '/obstacles', 10)
        
        self.sub = self.create_subscription(PointCloud2, '/zed2_left_camera/points', self.point_cloud_callback, 1)
        
        self.front = False
        self.left = False
        self.right = False
        self.front_threshold = 1.4
        self.front_side_threshold = 1.0
        self.side_threshold = 0.6
        self.side_threshold_front = 0.5
        self.leaf_size = 0.1
        
    def point_cloud_callback(self, cloud_msg):
        cloud = np.array(list(point_cloud2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True)))
        pcl_cloud = pcl.PointCloud_PointXYZ()
        pcl_cloud.from_array(cloud.astype(np.float32))
        
        # Apply Voxel Grid filtering to downsample the point cloud
        voxel_grid = pcl_cloud.make_voxel_grid_filter()
        voxel_grid.set_leaf_size(self.leaf_size, self.leaf_size, self.leaf_size)
        filtered_cloud = voxel_grid.filter()
        
        # Perform RANSAC segmentation to separate ground plane
        seg = filtered_cloud.make_segmenter()
        seg.set_optimize_coefficients(True)
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_distance_threshold(0.03)
        seg.set_axis(0, 0, 1)
        indices, coefficients = seg.segment()
        
        extract = filtered_cloud.extract(indices, negative=True)
        
        out_msg = point_cloud2.create_cloud_xyz32(cloud_msg.header, extract.to_array())
        self.filtered_pcl_publisher.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    pcl_node = PCL()
    rclpy.spin(pcl_node)
    pcl_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
