#!/usr/bin/env python3
"""
Launch file for LiDAR LD06 sensor integration
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    frame_id = LaunchConfiguration('frame_id', default='lidar_frame')
    parent_frame_id = LaunchConfiguration('parent_frame_id', default='base_link')

    x = LaunchConfiguration('x', default='0.0')
    y = LaunchConfiguration('y', default='0.0')
    z = LaunchConfiguration('z', default='0.0')
    roll = LaunchConfiguration('roll', default='0.0')
    pitch = LaunchConfiguration('pitch', default='0.0')
    yaw = LaunchConfiguration('yaw', default='0.0')
    
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

        DeclareLaunchArgument(
            'parent_frame_id',
            default_value='base_link',
            description='Parent TF frame for LiDAR static transform'
        ),

        DeclareLaunchArgument('x', default_value='0.0', description='LiDAR static TF x (meters)'),
        DeclareLaunchArgument('y', default_value='0.0', description='LiDAR static TF y (meters)'),
        DeclareLaunchArgument('z', default_value='0.0', description='LiDAR static TF z (meters)'),
        DeclareLaunchArgument('roll', default_value='0.0', description='LiDAR static TF roll (radians)'),
        DeclareLaunchArgument('pitch', default_value='0.0', description='LiDAR static TF pitch (radians)'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='LiDAR static TF yaw (radians)'),

        # Static TF: base_link -> lidar_frame (or whatever frame_id is set to)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_static_tf_pub',
            output='screen',
            arguments=[
                '--x', x,
                '--y', y,
                '--z', z,
                '--roll', roll,
                '--pitch', pitch,
                '--yaw', yaw,
                '--frame-id', parent_frame_id,
                '--child-frame-id', frame_id,
            ]
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
        
        # LiDAR processor node
        Node(
            package='robosort_sensors',
            executable='lidar_processor',
            name='lidar_processor',
            output='screen',
            parameters=[{
                'max_range': 12.0,
                'min_range': 0.02,
                'roi_angle': 60.0,
                'object_detection_threshold': 0.1,
            }]
        ),
        
        # Object localizer node
        Node(
            package='robosort_sensors',
            executable='object_localizer',
            name='object_localizer',
            output='screen',
            parameters=[{
                'camera_height': 0.3,
                'camera_tilt_angle': 30.0,
                'gripper_offset_x': 0.05,
            }]
        ),
    ])
