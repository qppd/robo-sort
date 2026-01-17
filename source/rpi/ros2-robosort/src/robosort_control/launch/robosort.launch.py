#!/usr/bin/env python3
"""
RoboSort Main Launch File
Launches LiDAR, motor control, TF broadcaster, and obstacle avoidance
For autonomous navigation without static maps
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    robosort_control_dir = get_package_share_directory('robosort_control')
    robosort_description_dir = get_package_share_directory('robosort_description')
    ldlidar_dir = get_package_share_directory('ldlidar_stl_ros2')
    
    rviz_config = os.path.join(robosort_control_dir, 'config', 'robosort.rviz')
    urdf_file = os.path.join(robosort_description_dir, 'urdf', 'robosort.urdf.xacro')
    
    # Declare launch arguments
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB0',
        description='LiDAR serial port'
    )
    
    arduino_port_arg = DeclareLaunchArgument(
        'arduino_port',
        default_value='/dev/ttyACM0',
        description='Arduino serial port'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )
    
    autonomous_arg = DeclareLaunchArgument(
        'autonomous',
        default_value='false',
        description='Enable autonomous wandering mode'
    )
    
    use_teleop_arg = DeclareLaunchArgument(
        'use_teleop',
        default_value='false',
        description='Launch teleop keyboard control'
    )
    
    # TF Broadcaster Node - handles odometry transform only
    # LiDAR transform now comes from robot_state_publisher via URDF
    tf_broadcaster_node = Node(
        package='robosort_control',
        executable='tf_broadcaster',
        name='tf_broadcaster',
        output='screen',
        parameters=[{
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'publish_rate': 50.0,
        }]
    )
    
    # Robot State Publisher - publishes robot URDF transforms
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'use_sim_time': False
        }]
    )

    # Joint State Publisher
    # Needed so robot_state_publisher will publish transforms for non-fixed joints
    # (e.g., continuous wheel joints). Without /joint_states, RViz can show a
    # "broken" model with overlapping/missing wheels.
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )
    
    # LD06 LiDAR driver node
    lidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='ld06_lidar',
        output='screen',
        parameters=[{
            'product_name': 'LDLiDAR_LD06',
            'topic_name': 'scan',
            'frame_id': 'lidar_link',
            'port_name': LaunchConfiguration('lidar_port'),
            'port_baudrate': 230400,
            'laser_scan_dir': True,
            'enable_angle_crop_func': False,
            'angle_crop_min': 0.0,
            'angle_crop_max': 0.0,
        }]
    )
    
    # Motor Controller Node
    motor_controller_node = Node(
        package='robosort_control',
        executable='motor_controller',
        name='motor_controller',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('arduino_port'),
            'baudrate': 9600,
            'timeout': 1.0,
            'max_speed': 255,
            'wheel_base': 0.2,
        }]
    )
    
    # Obstacle Avoidance Node
    obstacle_avoidance_node = Node(
        package='robosort_control',
        executable='obstacle_avoidance',
        name='obstacle_avoidance',
        output='screen',
        parameters=[{
            'min_obstacle_distance': 0.3,
            'warning_distance': 0.5,
            'max_linear_speed': 0.3,
            'max_angular_speed': 1.0,
            'front_angle': 60.0,
            'side_angle': 30.0,
            'enabled': True,
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
    
    # Teleop keyboard control (conditional)
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        condition=IfCondition(LaunchConfiguration('use_teleop'))
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(lidar_port_arg)
    ld.add_action(arduino_port_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(autonomous_arg)
    ld.add_action(use_teleop_arg)
    
    # Add nodes in order (TF first, robot state, then sensors, then control)
    ld.add_action(tf_broadcaster_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(lidar_node)
    ld.add_action(motor_controller_node)
    ld.add_action(obstacle_avoidance_node)
    ld.add_action(rviz_node)
    ld.add_action(teleop_node)
    
    return ld
