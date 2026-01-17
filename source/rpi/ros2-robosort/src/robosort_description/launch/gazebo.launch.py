#!/usr/bin/env python3
"""
Launch file for RoboSort robot in Gazebo simulation
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_robosort_description = get_package_share_directory('robosort_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # URDF file path
    urdf_file = os.path.join(pkg_robosort_description, 'urdf', 'robosort.urdf.xacro')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.1')
    world = LaunchConfiguration('world', default='empty')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_x_pose = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='Initial x position of the robot'
    )
    
    declare_y_pose = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Initial y position of the robot'
    )
    
    declare_z_pose = DeclareLaunchArgument(
        'z_pose',
        default_value='0.1',
        description='Initial z position of the robot'
    )
    
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='World file name (without .world extension)'
    )
    
    # Start Gazebo server
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )
    
    # Start Gazebo client
    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    
    # Robot state publisher
    start_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'use_sim_time': use_sim_time
        }]
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robosort',
        output='screen',
        arguments=[
            '-entity', 'robosort',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose
        ]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_x_pose,
        declare_y_pose,
        declare_z_pose,
        declare_world,
        start_gazebo_server,
        start_gazebo_client,
        start_robot_state_publisher,
        spawn_robot
    ])
