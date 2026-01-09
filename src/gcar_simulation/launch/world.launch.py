#!/usr/bin/env python3
"""Launch Gazebo with the GCAR city world."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate the launch description for the GCAR city world."""

    # Get the package share directory
    pkg_gcar_simulation = get_package_share_directory('gcar_simulation')

    # Path to the world file
    world_file = os.path.join(pkg_gcar_simulation, 'worlds', 'city.world')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')
    paused = LaunchConfiguration('paused', default='false')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to false to run headless'
    )

    declare_paused = DeclareLaunchArgument(
        'paused',
        default_value='false',
        description='Start Gazebo in a paused state'
    )

    # Launch Gazebo with the city world
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            world_file,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_gui,
        declare_paused,
        gazebo,
    ])

