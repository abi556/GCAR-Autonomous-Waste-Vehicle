#!/usr/bin/env python3
"""Launch file for GCAR arm control."""

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for arm manipulation."""

    # gazebo_ros2_control runs controller_manager in the robot namespace
    controller_manager = '/gcar/controller_manager'

    # Spawn joint_state_broadcaster (waits for controller_manager services)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', controller_manager,
        ],
        output='screen',
    )

    # Spawn arm_controller (waits for controller_manager services)
    arm_controller_spawner = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'arm_controller',
                    '--controller-manager', controller_manager,
                ],
                output='screen',
            )
        ]
    )

    # Start helper node (services -> FollowJointTrajectory action)
    arm_controller_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='gcar_manipulation',
                executable='arm_controller',
                name='arm_controller',
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        arm_controller_node,
    ])

