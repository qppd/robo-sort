#!/usr/bin/env python3
"""
Launch file for RoboSort complete system with Gazebo simulation
Includes: Gazebo, RViz, motor controller, obstacle avoidance, TF broadcaster
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_robosort_description = get_package_share_directory('robosort_description')
    pkg_robosort_control = get_package_share_directory('robosort_control')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Include Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robosort_description, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # RViz config file
    rviz_config_file = os.path.join(pkg_robosort_description, 'rviz', 'view_robot.rviz')
    
    # RViz
    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Obstacle avoidance node
    start_obstacle_avoidance = Node(
        package='robosort_control',
        executable='obstacle_avoidance',
        name='obstacle_avoidance',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'obstacle_distance_threshold': 0.5,
            'stop_distance': 0.3
        }]
    )
    
    # Teleop keyboard control
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e'
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        gazebo_launch,
        start_rviz,
        start_obstacle_avoidance,
        teleop_node
    ])
