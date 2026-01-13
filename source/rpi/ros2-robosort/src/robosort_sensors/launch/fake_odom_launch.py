#!/usr/bin/env python3
"""
Launch file for fake odometry publisher
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robosort_sensors',
            executable='fake_odom',
            name='fake_odom_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
    ])
