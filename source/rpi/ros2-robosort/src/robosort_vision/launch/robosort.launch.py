#!/usr/bin/env python3
"""
RoboSort Main Launch File
Launches all nodes for the waste segregation system
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('robosort_vision')
    rviz_config = os.path.join(pkg_dir, 'config', 'robosort.rviz')
    
    # Declare launch arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='yolov8n.pt',
        description='Path to YOLO model file'
    )
    
    camera_source_arg = DeclareLaunchArgument(
        'camera_source',
        default_value='usb0',
        description='Camera source: usb0, picamera0, or device path'
    )
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Arduino serial port'
    )
    
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='YOLO confidence threshold'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    # YOLO Detector Node
    yolo_detector_node = Node(
        package='robosort_vision',
        executable='yolo_detector',
        name='yolo_detector',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'camera_source': LaunchConfiguration('camera_source'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'camera_topic': '/camera/image_raw',
            'detection_topic': '/robosort/detections',
            'annotated_image_topic': '/robosort/annotated_image',
            'publish_rate': 30.0
        }]
    )
    
    # Arduino Serial Bridge Node
    arduino_serial_node = Node(
        package='robosort_vision',
        executable='arduino_serial',
        name='arduino_serial',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': 9600,
            'timeout': 1.0,
            'ultrasonic_publish_rate': 10.0
        }]
    )
    
    # Waste Segregation Controller Node
    controller_node = Node(
        package='robosort_vision',
        executable='waste_segregation_controller',
        name='waste_segregation_controller',
        output='screen',
        parameters=[{
            'bin_capacity_threshold': 15.0,
            'pickup_height': 100.0,
            'drop_height': 0.0
        }]
    )
    
    # RViz Node (conditional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(model_path_arg)
    ld.add_action(camera_source_arg)
    ld.add_action(serial_port_arg)
    ld.add_action(confidence_threshold_arg)
    ld.add_action(use_rviz_arg)
    
    # Add nodes
    ld.add_action(yolo_detector_node)
    ld.add_action(arduino_serial_node)
    ld.add_action(controller_node)
    ld.add_action(rviz_node)
    
    return ld
