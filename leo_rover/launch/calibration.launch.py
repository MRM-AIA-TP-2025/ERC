from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_calibration',
            executable='cameracalibrator',
            name='camera_calibration',
            output='screen',
            parameters=[{
                'camera_frame': 'zed2_left_camera_optical_frame',  # Adjust to your frame
                'image': '/zed2_left_camera/image_raw',
                'camera_info': '/zed2_left_camera/camera_info',
                'square_size': 0.02  # Adjust to your checkerboard square size
            }],
            remappings=[
                ('/image', '/zed2_left_camera/image_raw'),
                ('/camera_info', '/zed2_left_camera/camera_info')
            ]
        )
    ])

