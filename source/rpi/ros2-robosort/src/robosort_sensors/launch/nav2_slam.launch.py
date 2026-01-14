#!/usr/bin/env python3
"""Launch Nav2 in SLAM mode (no AMCL).

Why:
- Your TF tree currently only has `map -> lidar_frame` (from slam_toolbox), and
  no `odom` frame at all.
- Nav2's AMCL stack requires a valid `map -> odom -> base_link` style TF chain.
- Running slam_toolbox + Nav2 (with AMCL) at the same time will also create
  conflicting localization sources.

This launch file starts nav2_bringup with slam:=True so Nav2 uses slam_toolbox
instead of AMCL.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=[
                FindPackageShare('robosort_sensors'),
                '/config/nav2_params.yaml',
            ],
            description='Path to Nav2 parameters YAML file',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('nav2_bringup'),
                '/launch/navigation_launch.py',  # Changed: just nav stack, no map/localization
            ]),
            launch_arguments={
                'params_file': params_file,
            }.items(),
        ),
    ])
