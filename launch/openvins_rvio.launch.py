#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('terrain_mapping_drone_control')
    default_config = os.path.join(pkg_share, 'config', 'openvins_mars_estimator.yaml')

    config_path = LaunchConfiguration('config_path')
    start_imu_bridge = LaunchConfiguration('start_imu_bridge')
    start_evaluator = LaunchConfiguration('start_evaluator')
    start_rqt_plots = LaunchConfiguration('start_rqt_plots')
    rvio_start_state = LaunchConfiguration('rvio_start_state')

    imu_bridge = Node(
        package='terrain_mapping_drone_control',
        executable='px4_imu_bridge',
        name='px4_imu_bridge',
        parameters=[{
            'use_sim_time': True,
            'px4_sensor_combined_topic': '/fmu/out/sensor_combined',
            'imu_topic': '/openvins/imu',
            'frame_id': 'imu',
        }],
        condition=IfCondition(start_imu_bridge),
        output='screen',
    )

    openvins = Node(
        package='terrain_mapping_drone_control',
        executable='openvins_on_state_launcher',
        name='openvins_on_state_launcher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'mission_state_topic': '/mission/state',
            'start_on_state': rvio_start_state,
            'config_path': config_path,
            'verbosity': 'INFO',
            'topic_imu': '/openvins/imu',
            'topic_camera0': '/drone/down_mono',
            'topic_range': '/drone/down_rangefinder',
            'use_stereo': False,
            'max_cameras': 1,
            'range_feature_enable': True,
        }],
    )

    evaluator = Node(
        package='terrain_mapping_drone_control',
        executable='rvio_evaluator_node',
        name='rvio_evaluator_node',
        parameters=[{
            'use_sim_time': True,
            'rvio_odom_topic': '/rvio/odometry',
            'ground_truth_topic': '/sim/ground_truth/vehicle_odometry',
            'range_topic': '/drone/down_rangefinder',
            'write_csv': False,
            'mission_state_topic': '/mission/state',
            'start_on_mission_state': rvio_start_state,
        }],
        condition=IfCondition(start_evaluator),
        output='screen',
    )

    rqt_plots = Node(
        package='terrain_mapping_drone_control',
        executable='rvio_rqt_plot_launcher',
        name='rvio_rqt_plot_launcher',
        parameters=[{
            'mission_state_topic': '/mission/state',
            'open_on_state': rvio_start_state,
            'open_x_plot': True,
            'open_y_plot': True,
            'open_debug_image': True,
        }],
        condition=IfCondition(start_rqt_plots),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value=default_config),
        DeclareLaunchArgument('start_imu_bridge', default_value='true'),
        DeclareLaunchArgument('start_evaluator', default_value='true'),
        DeclareLaunchArgument('start_rqt_plots', default_value='true'),
        DeclareLaunchArgument('rvio_start_state', default_value='SURVEY'),
        imu_bridge,
        openvins,
        evaluator,
        rqt_plots,
    ])
