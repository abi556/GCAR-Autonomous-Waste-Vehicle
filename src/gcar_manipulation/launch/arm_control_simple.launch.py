#!/usr/bin/env python3
"""Simplified launch file for GCAR arm control using spawner."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for arm manipulation."""
    
    # Get controller config file
    pkg_gcar_description = get_package_share_directory('gcar_description')
    controller_config = os.path.join(pkg_gcar_description, 'config', 'arm_controllers.yaml')
    
    # Spawn joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    # Spawn arm_controller (with delay)
    arm_controller_spawner = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['arm_controller', '--controller-manager', '/controller_manager'],
                output='screen',
            )
        ]
    )
    
    # Start arm controller node (with delay to ensure controllers are loaded)
    arm_controller_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='gcar_manipulation',
                executable='arm_controller',
                name='arm_controller_node',
                output='screen',
                parameters=[],
            )
        ]
    )
    
    return LaunchDescription([
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        arm_controller_node,
    ])

