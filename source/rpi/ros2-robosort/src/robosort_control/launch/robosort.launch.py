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
    nav2_params_file = os.path.join(robosort_control_dir, 'config', 'nav2_params.yaml')
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
    
    use_nav2_arg = DeclareLaunchArgument(
        'use_nav2',
        default_value='false',
        description='Launch Nav2 navigation stack'
    )
    
    # RF2O Laser Odometry - provides odom->base_footprint transform from lidar scan matching
    # This is the PRIMARY odometry source for RoboSort (uses laser scan matching)
    # Note: RViz may show "Waiting for transform" for 2-3 seconds until RF2O accumulates enough scans
    rf2o_params_file = os.path.join(robosort_control_dir, 'config', 'rf2o_params.yaml')
    rf2o_laser_odometry_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[rf2o_params_file]
    )
    
    # Robot State Publisher - publishes robot URDF transforms
    # Publishes: base_footprint -> base_link -> lidar_link (and all other links)
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
            'topic_name': '/scan',  # Changed to /scan with leading slash for consistency
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
            'enabled': False,  # Disabled by default, enable via service or autonomous mode
        }]
    )
    
    # RViz Node (conditional)
    # Note: Removed xterm prefix as it requires xterm to be installed
    # RViz will launch in the same terminal. Use launch_rviz.sh for separate window if needed
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
    
    # Nav2 Navigation Stack (conditional)
    # Nav2 Jazzy bringup includes docking_server by default, which fails if no dock
    # plugins are configured. RoboSort doesn't use docking, so we launch Nav2 without it.
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robosort_control_dir, 'launch', 'nav2_no_docking.launch.py')
        ),
        launch_arguments={
            'namespace': '',
            'use_sim_time': 'false',
            'params_file': nav2_params_file,
            'autostart': 'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_nav2'))
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(lidar_port_arg)
    ld.add_action(arduino_port_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(autonomous_arg)
    ld.add_action(use_teleop_arg)
    ld.add_action(use_nav2_arg)
    
    # Add nodes in order
    # Start robot_state_publisher first to publish static URDF transforms
    # Then start lidar and RF2O for odometry
    # Note: RViz may show "Waiting for transform" for 2-3 seconds until RF2O initializes
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(lidar_node)
    ld.add_action(rf2o_laser_odometry_node)  # Primary odometry source
    ld.add_action(motor_controller_node)
    ld.add_action(obstacle_avoidance_node)
    ld.add_action(rviz_node)
    ld.add_action(teleop_node)
    ld.add_action(nav2_launch)
    
    return ld
