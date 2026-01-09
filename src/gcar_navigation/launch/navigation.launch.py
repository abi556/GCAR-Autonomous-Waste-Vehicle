#!/usr/bin/env python3
"""Launch file for GCAR Navigation with SLAM Toolbox and Nav2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description for GCAR navigation."""

    # Get package directories
    pkg_gcar_navigation = get_package_share_directory('gcar_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    # Paths to config files
    nav2_params_file = os.path.join(pkg_gcar_navigation, 'params', 'nav2_params.yaml')
    slam_params_file = os.path.join(pkg_gcar_navigation, 'config', 'slam_toolbox_params.yaml')
    rviz_config_file = os.path.join(pkg_gcar_navigation, 'rviz', 'nav2_view.rviz')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam = LaunchConfiguration('slam', default='true')
    map_file = LaunchConfiguration('map', default='')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_slam = DeclareLaunchArgument(
        'slam',
        default_value='true',
        description='Whether to run SLAM (true) or localization only (false)'
    )

    declare_map = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file to load (for localization mode)'
    )

    # SLAM Toolbox Node (online async mode)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
    )

    # Nav2 Bringup
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
        }.items()
    )

    # RViz2 with Navigation display
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_slam,
        declare_map,
        slam_toolbox_node,
        nav2_bringup_launch,
        rviz2_node,
    ])

