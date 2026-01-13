#!/usr/bin/env python3
"""
Complete RoboSort Nav2 Visualization Launch
Launches robot description, odometry, Nav2 SLAM, and RViz
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    desc_pkg = get_package_share_directory('robosort_description')
    sensors_pkg = get_package_share_directory('robosort_sensors')

    # File paths
    urdf_file = os.path.join(desc_pkg, 'urdf', 'robosort_arm.urdf.xacro')
    nav2_rviz_config = os.path.join(sensors_pkg, 'config', 'slam_nav2.rviz')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    slam_params_file = LaunchConfiguration('slam_params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(sensors_pkg, 'config', 'slam_params.yaml'),
            description='SLAM Toolbox parameters file'
        ),

        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(sensors_pkg, 'config', 'nav2_params.yaml'),
            description='Nav2 parameters file'
        ),

        # Robot State Publisher (publish TF tree)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_file]),
                'use_sim_time': use_sim_time
            }]
        ),

        # Joint State Publisher (for robot visualization)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # SLAM Toolbox (creates map and provides localization)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('slam_toolbox'),
                    'launch',
                    'online_async_launch.py'
                ])
            ),
            launch_arguments={
                'slam_params_file': slam_params_file,
                'use_sim_time': use_sim_time
            }.items()
        ),

        # Nav2 Bringup (navigation stack)
        TimerAction(
            period=5.0,  # Wait for SLAM to initialize
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('nav2_bringup'),
                            'launch',
                            'bringup_launch.py'
                        ])
                    ),
                    launch_arguments={
                        'slam': 'True',
                        'params_file': nav2_params_file,
                        'use_sim_time': use_sim_time
                    }.items()
                )
            ]
        ),

        # RViz with Nav2 configuration
        TimerAction(
            period=10.0,  # Wait for everything to start
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2_nav2',
                    output='screen',
                    arguments=['-d', nav2_rviz_config],
                    parameters=[{'use_sim_time': use_sim_time}]
                )
            ]
        ),
    ])