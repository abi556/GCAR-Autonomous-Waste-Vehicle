#!/usr/bin/env python3
"""Launch file to spawn GCAR robot into Gazebo."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Generate the launch description for spawning GCAR in Gazebo."""

    # Get package directories
    pkg_gcar_description = get_package_share_directory('gcar_description')
    pkg_gcar_simulation = get_package_share_directory('gcar_simulation')

    # Paths to files
    urdf_file = os.path.join(pkg_gcar_description, 'urdf', 'gcar.urdf.xacro')
    controller_config = os.path.join(pkg_gcar_description, 'config', 'arm_controllers.yaml')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Spawn position - at origin on the road (z=0.1 to account for road surface)
    # NOTE: Robot must spawn near (0,0) so SLAM map includes robot position for Nav2 to work
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.1')
    yaw = LaunchConfiguration('yaw', default='0.0')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_x_pose = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='X position for spawning the robot'
    )

    declare_y_pose = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Y position for spawning the robot'
    )

    declare_z_pose = DeclareLaunchArgument(
        'z_pose',
        default_value='0.1',
        description='Z position for spawning the robot (accounts for road surface)'
    )

    declare_yaw = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Yaw orientation for spawning the robot'
    )

    # Robot description from xacro (pass controller config as argument)
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' controller_config_file:=', controller_config]),
        value_type=str
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='gcar',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # Spawn the robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_gcar',
        output='screen',
        arguments=[
            '-topic', '/gcar/robot_description',
            '-entity', 'gcar',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw
        ]
    )

    # Include the world launch file
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gcar_simulation, 'launch', 'world.launch.py')
        )
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_x_pose,
        declare_y_pose,
        declare_z_pose,
        declare_yaw,
        world_launch,
        robot_state_publisher,
        spawn_robot,
    ])

