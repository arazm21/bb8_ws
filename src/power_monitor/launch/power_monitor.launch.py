#!/usr/bin/env python3
"""
Launch file for power monitoring nodes
Starts combined publisher and power monitor subscriber
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level for all nodes'
        ),

        # Combined Publisher Node (UPS + RPi in one)
        Node(
            package='power_monitor',
            executable='combined_publisher',
            name='combined_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),

        # Power Monitor Subscriber Node
        Node(
            package='power_monitor',
            executable='power_monitor_subscriber',
            name='power_monitor_subscriber',
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),
    ])