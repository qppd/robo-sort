#!/usr/bin/env python3
"""
Gazebo Simulation Launch File (launch_sim.launch.py)
Following Articulated Robotics Tutorial Pattern

This launch file:
1. Includes rsp.launch.py with use_sim_time:=true
2. Launches Gazebo with the specified world file
3. Spawns the robot entity from /robot_description topic

Usage:
  ros2 launch robosort_description launch_sim.launch.py
  ros2 launch robosort_description launch_sim.launch.py world:=./src/robosort_description/worlds/robosort_warehouse.world

After launching:
  - Use teleop_twist_keyboard to drive the robot:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
  
  - View lidar data:
    ros2 topic echo /scan
  
  - View odometry:
    ros2 topic echo /odom
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_robosort_description = get_package_share_directory('robosort_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # Default world file path
    default_world = os.path.join(pkg_robosort_description, 'worlds', 'robosort_warehouse.world')
    
    # RViz config file
    rviz_config_file = os.path.join(pkg_robosort_description, 'rviz', 'simulation.rviz')
    
    # Launch configuration variables
    world = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('use_rviz')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    
    # Declare launch arguments
    declare_world = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Full path to the world file to load'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz2'
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
        default_value='0.25',
        description='Initial z position of the robot (should be high enough to avoid collision)'
    )
    
    # ============================================
    # 1. Include Robot State Publisher (with sim_time)
    # ============================================
    # This publishes the robot URDF to /robot_description
    # and broadcasts transforms for fixed joints
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_robosort_description, 'launch', 'rsp.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    # ============================================
    # 2. Launch Gazebo
    # ============================================
    # Start Gazebo server (physics simulation)
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ]),
        launch_arguments={
            'world': world,
            'verbose': 'true'
        }.items()
    )
    
    # Start Gazebo client (GUI)
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ]),
        launch_arguments={
            'verbose': 'true'
        }.items()
    )
    
    # ============================================
    # 3. Spawn Robot in Gazebo
    # ============================================
    # The spawn_entity node reads from /robot_description
    # and spawns the robot model in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robosort',
        output='screen',
        arguments=[
            '-entity', 'robosort',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-timeout', '120.0'
        ]
    )
    
    # ============================================
    # 4. Launch RViz2 (Optional)
    # ============================================
    # Delayed start to ensure robot_description is published
    rviz = TimerAction(
        period=3.0,  # Wait 3 seconds for everything to start
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_file],
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(use_rviz)
            )
        ]
    )
    
    return LaunchDescription([
        # Declare arguments
        declare_world,
        declare_use_rviz,
        declare_x_pose,
        declare_y_pose,
        declare_z_pose,
        
        # Launch robot state publisher
        rsp,
        
        # Launch Gazebo
        gazebo_server,
        gazebo_client,
        
        # Spawn robot
        spawn_entity,
        
        # Launch RViz (delayed)
        rviz,
    ])
