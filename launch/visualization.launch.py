#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """Generate launch description for drone visualization."""
    
    # Get the package share directory
    pkg_share = get_package_share_directory('terrain_mapping_drone_control')
    
    rviz_mode = LaunchConfiguration('rviz_mode')
    rviz_config = PythonExpression([
        "'",
        os.path.join(pkg_share, 'config', 'down_mono_debug.rviz'),
        "' if '",
        rviz_mode,
        "' == 'down_mono' else '",
        os.path.join(pkg_share, 'config', 'drone_viz.rviz'),
        "'",
    ])
    
    # Launch RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    # Launch pose visualizer
    pose_visualizer = Node(
        package='terrain_mapping_drone_control',
        executable='pose_visualizer',
        name='pose_visualizer',
        output='screen'
    )

    return LaunchDescription([
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        DeclareLaunchArgument(
            'rviz_mode',
            default_value='default',
            description='Set to down_mono to show the downward-facing mono camera debug view'),
        pose_visualizer,
        rviz_node
    ])
