from setuptools import find_packages, setup

package_name = 'leo_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'opencv-python',
        'opencv-contrib-python',
        'numpy',
        'cv_bridge',
    ],
    zip_safe=True,
    maintainer='aditi',
    maintainer_email='aditi.pers@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pdu_node = leo_nodes.pdu_node:main',
            'ground_truth_filter = leo_nodes.ground_truth_filter:main',
            'odom_compat_node = leo_nodes.odom_compat_node:main',
            'zed2_imu_transform_broadcaster = leo_nodes.zed2_imu_transform_broadcaster:main',
            'cmd_vel_relay = leo_nodes.cmd_vel_relay:main',
            'zed2_imu_transform_publisher = leo_nodes.zed2_imu_transform_publisher:main',
            'navigation_controller = leo_nodes.navigation_control:main',
            'odom_offset_node = leo_nodes.odom_offset_node:main',  # Add this line
            'aruco = leo_nodes.aruco:main',
            'goal_setter_node=leo_nodes.goal_setter_node:main',
        ],
    },
)

