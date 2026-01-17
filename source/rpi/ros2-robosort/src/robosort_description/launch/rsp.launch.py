#!/usr/bin/env python3
"""
Robot State Publisher Launch File (rsp.launch.py)
Following Articulated Robotics Tutorial Pattern

This launch file starts the robot_state_publisher node which:
- Publishes the robot URDF to /robot_description topic
- Publishes TF transforms for fixed joints
- Uses sim_time when running with Gazebo

Usage:
  ros2 launch robosort_description rsp.launch.py
  ros2 launch robosort_description rsp.launch.py use_sim_time:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('robosort_description')
    
    # URDF file path
    urdf_file = os.path.join(pkg_dir, 'urdf', 'robosort.urdf.xacro')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Robot state publisher node
    # This node publishes:
    # - /robot_description (the full URDF)
    # - /tf_static (transforms for fixed joints)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'use_sim_time': use_sim_time
        }]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher,
    ])
