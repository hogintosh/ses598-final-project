#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, UnsetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the RVIO proof-of-concept estimator against existing sim topics."""

    use_sim_time = LaunchConfiguration('use_sim_time')
    image_topic = LaunchConfiguration('image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    range_topic = LaunchConfiguration('range_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    px4_sensor_combined_topic = LaunchConfiguration('px4_sensor_combined_topic')
    px4_vehicle_attitude_topic = LaunchConfiguration('px4_vehicle_attitude_topic')
    ground_truth_topic = LaunchConfiguration('ground_truth_topic')
    ground_truth_error_z_mode = LaunchConfiguration('ground_truth_error_z_mode')
    estimator_mode = LaunchConfiguration('estimator_mode')
    mission_state_topic = LaunchConfiguration('mission_state_topic')
    start_on_mission_state = LaunchConfiguration('start_on_mission_state')
    start_evaluator = LaunchConfiguration('start_evaluator')
    write_eval_csv = LaunchConfiguration('write_eval_csv')
    use_discovery_server = LaunchConfiguration('use_discovery_server')
    discovery_server = LaunchConfiguration('discovery_server')
    discovery_range = LaunchConfiguration('discovery_range')
    range_feature_use_attitude_projection = LaunchConfiguration(
        'range_feature_use_attitude_projection'
    )
    range_feature_lidar_projection_radius_px = LaunchConfiguration(
        'range_feature_lidar_projection_radius_px'
    )
    range_feature_reprojection_gate_px = LaunchConfiguration(
        'range_feature_reprojection_gate_px'
    )

    rvio = Node(
        package='terrain_mapping_drone_control',
        executable='rvio_poc_node',
        name='rvio_poc_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'image_topic': image_topic,
            'camera_info_topic': camera_info_topic,
            'range_topic': range_topic,
            'imu_topic': imu_topic,
            'px4_sensor_combined_topic': px4_sensor_combined_topic,
            'px4_vehicle_attitude_topic': px4_vehicle_attitude_topic,
            'ground_truth_topic': ground_truth_topic,
            'ground_truth_error_z_mode': ground_truth_error_z_mode,
            'estimator_mode': estimator_mode,
            'mission_state_topic': mission_state_topic,
            'start_on_mission_state': start_on_mission_state,
            'range_feature_use_attitude_projection': range_feature_use_attitude_projection,
            'range_feature_lidar_projection_radius_px': range_feature_lidar_projection_radius_px,
            'range_feature_reprojection_gate_px': range_feature_reprojection_gate_px,
        }],
        output='screen',
    )

    evaluator = Node(
        package='terrain_mapping_drone_control',
        executable='rvio_evaluator_node',
        name='rvio_evaluator_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'rvio_odom_topic': '/rvio/odometry',
            'ground_truth_topic': ground_truth_topic,
            'range_topic': range_topic,
            'write_csv': write_eval_csv,
            'mission_state_topic': mission_state_topic,
            'start_on_mission_state': start_on_mission_state,
        }],
        condition=IfCondition(start_evaluator),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_discovery_server',
            default_value='false',
            description='Use a Fast DDS discovery server instead of ROS_AUTOMATIC_DISCOVERY_RANGE.',
        ),
        DeclareLaunchArgument(
            'discovery_server',
            default_value='127.0.0.1:11811',
            description='Fast DDS discovery server address used when use_discovery_server is true.',
        ),
        DeclareLaunchArgument(
            'discovery_range',
            default_value='LOCALHOST',
            description='ROS_AUTOMATIC_DISCOVERY_RANGE used when use_discovery_server is false.',
        ),
        SetEnvironmentVariable(
            'ROS_DISCOVERY_SERVER',
            discovery_server,
            condition=IfCondition(use_discovery_server),
        ),
        SetEnvironmentVariable(
            'ROS_SUPER_CLIENT',
            'True',
            condition=IfCondition(use_discovery_server),
        ),
        UnsetEnvironmentVariable(
            'ROS_AUTOMATIC_DISCOVERY_RANGE',
            condition=IfCondition(use_discovery_server),
        ),
        UnsetEnvironmentVariable(
            'ROS_DISCOVERY_SERVER',
            condition=UnlessCondition(use_discovery_server),
        ),
        UnsetEnvironmentVariable(
            'ROS_SUPER_CLIENT',
            condition=UnlessCondition(use_discovery_server),
        ),
        SetEnvironmentVariable(
            'ROS_DOMAIN_ID',
            '15',
            condition=UnlessCondition(use_discovery_server),
        ),
        SetEnvironmentVariable(
            'ROS_AUTOMATIC_DISCOVERY_RANGE',
            discovery_range,
            condition=UnlessCondition(use_discovery_server),
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo /clock for estimator output stamps.',
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/drone/down_mono',
            description='Downward monocular camera image topic.',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/drone/down_mono/camera_info',
            description='Downward monocular camera calibration topic.',
        ),
        DeclareLaunchArgument(
            'range_topic',
            default_value='/drone/down_rangefinder',
            description='Downward 1D lidar/rangefinder LaserScan topic.',
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/drone/imu',
            description='Optional sensor_msgs/Imu topic if Gazebo IMU is bridged.',
        ),
        DeclareLaunchArgument(
            'px4_sensor_combined_topic',
            default_value='/fmu/out/sensor_combined',
            description='PX4 SensorCombined IMU topic used by the current sim.',
        ),
        DeclareLaunchArgument(
            'px4_vehicle_attitude_topic',
            default_value='/fmu/out/vehicle_attitude',
            description='PX4 attitude topic used for pitch/roll/yaw-aware visual compensation.',
        ),
        DeclareLaunchArgument(
            'ground_truth_topic',
            default_value='/sim/ground_truth/vehicle_odometry',
            description='Gazebo ground-truth odometry topic for error reporting only.',
        ),
        DeclareLaunchArgument(
            'ground_truth_error_z_mode',
            default_value='ignore',
            description='z field for /rvio/ground_truth_error: ignore, raw, or zero.',
        ),
        DeclareLaunchArgument(
            'estimator_mode',
            default_value='range_feature_3d',
            description='RVIO estimator mode: range_feature_3d full 3D translational range-feature pose estimator or direct visual odometry fusion.',
        ),
        DeclareLaunchArgument(
            'mission_state_topic',
            default_value='/mission/state',
            description='Mission phase topic used to optionally gate RVIO startup.',
        ),
        DeclareLaunchArgument(
            'start_on_mission_state',
            default_value='',
            description='Mission state that resets/starts RVIO and metrics; empty starts immediately.',
        ),
        DeclareLaunchArgument(
            'range_feature_use_attitude_projection',
            default_value='true',
            description='Project the lidar beam into the camera image using configured sensor extrinsics before seeding range features.',
        ),
        DeclareLaunchArgument(
            'range_feature_lidar_projection_radius_px',
            default_value='24.0',
            description='Pixel radius around projected lidar beam used for range-feature seeding.',
        ),
        DeclareLaunchArgument(
            'range_feature_reprojection_gate_px',
            default_value='8.0',
            description='Maximum range-feature reprojection residual in pixels allowed into the EKF update.',
        ),
        DeclareLaunchArgument(
            'start_evaluator',
            default_value='true',
            description='Start the RVIO relative-frame accuracy evaluator.',
        ),
        DeclareLaunchArgument(
            'write_eval_csv',
            default_value='false',
            description='Also write RVIO evaluator metrics to ~/.ros/rvio_eval; rqt_plot topics are always published.',
        ),
        rvio,
        evaluator,
    ])
