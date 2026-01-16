#!/usr/bin/env python3
"""
Launch file for LiDAR LD06 sensor - Minimal version
Only launches LiDAR driver and TF broadcaster for visualization
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    frame_id = LaunchConfiguration('frame_id', default='lidar_frame')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='LiDAR LD06 serial port'
        ),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value='lidar_frame',
            description='TF frame ID for LiDAR'
        ),
        
        # TF Broadcaster - handles odom -> base_link -> lidar_frame
        Node(
            package='robosort_control',
            executable='tf_broadcaster',
            name='tf_broadcaster',
            output='screen',
            parameters=[{
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'lidar_frame': frame_id,
                'publish_rate': 50.0,
                'lidar_x': 0.0,
                'lidar_y': 0.0,
                'lidar_z': 0.1,
            }]
        ),
        
        # LD06 LiDAR driver node
        Node(
            package='ldlidar_stl_ros2',
            executable='ldlidar_stl_ros2_node',
            name='ld06_lidar',
            output='screen',
            parameters=[{
                'product_name': 'LDLiDAR_LD06',
                'topic_name': 'scan',
                'frame_id': frame_id,
                'port_name': serial_port,
                'port_baudrate': 230400,
                'laser_scan_dir': True,
                'enable_angle_crop_func': False,
                'angle_crop_min': 0.0,
                'angle_crop_max': 0.0,
            }]
        ),
    ])
