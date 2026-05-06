#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
    UnsetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    """Generate launch description for cylinder landing mission."""
    
    # Get the package share directory
    pkg_share = get_package_share_directory('terrain_mapping_drone_control')
        
    # Set Gazebo model and resource paths
    gz_model_path = os.path.join(pkg_share, 'models')

    # Add launch arguments
    px4_autopilot_path = LaunchConfiguration('px4_autopilot_path')
    qgc_host_ip = LaunchConfiguration('qgc_host_ip')
    qgc_udp_port = LaunchConfiguration('qgc_udp_port')
    headless = LaunchConfiguration('headless')
    start_gazebo = LaunchConfiguration('start_gazebo')
    start_px4 = LaunchConfiguration('start_px4')
    start_microxrce_agent = LaunchConfiguration('start_microxrce_agent')
    microxrce_agent_port = LaunchConfiguration('microxrce_agent_port')
    px4_model = LaunchConfiguration('px4_model')
    px4_world = LaunchConfiguration('px4_world')
    px4_make_target = LaunchConfiguration('px4_make_target')
    px4_model_pose = LaunchConfiguration('px4_model_pose')
    model_instance = LaunchConfiguration('model_instance')
    spawn_cylinders = LaunchConfiguration('spawn_cylinders')
    start_rvio = LaunchConfiguration('start_rvio')
    start_rvio_evaluator = LaunchConfiguration('start_rvio_evaluator')
    start_rvio_rqt_plots = LaunchConfiguration('start_rvio_rqt_plots')
    write_eval_csv = LaunchConfiguration('write_eval_csv')
    rvio_estimator_mode = LaunchConfiguration('rvio_estimator_mode')
    rvio_backend = LaunchConfiguration('rvio_backend')
    use_discovery_server = LaunchConfiguration('use_discovery_server')
    discovery_server = LaunchConfiguration('discovery_server')
    discovery_range = LaunchConfiguration('discovery_range')
    mission_mode = LaunchConfiguration('mission_mode')
    hover_relative_height = LaunchConfiguration('hover_relative_height')
    require_preflight_checks = LaunchConfiguration('require_preflight_checks')
    rvio_test_agl = LaunchConfiguration('rvio_test_agl')
    rvio_test_climb_height = LaunchConfiguration('rvio_test_climb_height')
    rvio_test_motion_profile = LaunchConfiguration('rvio_test_motion_profile')
    rvio_test_axis = LaunchConfiguration('rvio_test_axis')
    rvio_test_axis_distance = LaunchConfiguration('rvio_test_axis_distance')
    rvio_test_axis_speed = LaunchConfiguration('rvio_test_axis_speed')
    rvio_start_state = LaunchConfiguration('rvio_start_state')
    rvio_flow_x_sign = LaunchConfiguration('rvio_flow_x_sign')
    rvio_flow_y_sign = LaunchConfiguration('rvio_flow_y_sign')
    rvio_camera_mount_roll_rad = LaunchConfiguration('rvio_camera_mount_roll_rad')
    rvio_camera_mount_pitch_rad = LaunchConfiguration('rvio_camera_mount_pitch_rad')
    rvio_camera_mount_yaw_rad = LaunchConfiguration('rvio_camera_mount_yaw_rad')
    rvio_range_feature_use_attitude_projection = LaunchConfiguration(
        'rvio_range_feature_use_attitude_projection'
    )
    rvio_range_feature_lidar_projection_radius_px = LaunchConfiguration(
        'rvio_range_feature_lidar_projection_radius_px'
    )
    rvio_range_feature_reprojection_gate_px = LaunchConfiguration(
        'rvio_range_feature_reprojection_gate_px'
    )
    rvio_camera_body_pitch_rad = LaunchConfiguration('rvio_camera_body_pitch_rad')
    rvio_lidar_body_pitch_rad = LaunchConfiguration('rvio_lidar_body_pitch_rad')
    rvio_lidar_body_roll_rad = LaunchConfiguration('rvio_lidar_body_roll_rad')
    rvio_max_visual_velocity_mps = LaunchConfiguration('rvio_max_visual_velocity_mps')
    rvio_max_range_feature_velocity_mps = LaunchConfiguration(
        'rvio_max_range_feature_velocity_mps'
    )
    rvio_max_range_feature_pixel_rate_pxps = LaunchConfiguration(
        'rvio_max_range_feature_pixel_rate_pxps'
    )

    # Start Gazebo separately so PX4 can attach to the already-running world.
    # Gazebo must advance briefly for PX4's world-ready check to pass; a timer
    # pauses it again before the operator proceeds from QGroundControl.
    gazebo = ExecuteProcess(
        cmd=[
            'bash', '-lc',
            [
                'export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:',
                px4_autopilot_path, '/Tools/simulation/gz/models:',
                px4_autopilot_path, '/Tools/simulation/gz/worlds" && ',
                'if [ "', headless, '" = "1" ] || [ "', headless, '" = "true" ]; then ',
                'gz sim --verbose=1 -r -s "', px4_autopilot_path, '/Tools/simulation/gz/worlds/', px4_world, '.sdf"; ',
                'else ',
                'gz sim --verbose=1 -r "', px4_autopilot_path, '/Tools/simulation/gz/worlds/', px4_world, '.sdf"; ',
                'fi',
            ],
        ],
        condition=IfCondition(start_gazebo),
        output='screen'
    )

    microxrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', microxrce_agent_port],
        condition=IfCondition(start_microxrce_agent),
        output='screen'
    )

    # Launch PX4 SITL with the selected model/world and route MAVLink to QGC on the Mac host.
    px4_sitl = ExecuteProcess(
        cmd=[
            'bash', '-lc',
            [
                'export PX4_GZ_MODEL=', px4_model, ' && ',
                'export PX4_GZ_WORLD=', px4_world, ' && ',
                'export PX4_GZ_MODEL_POSE=', px4_model_pose, ' && ',
                'if [ "', start_gazebo, '" = "1" ] || [ "', start_gazebo, '" = "true" ]; then ',
                'export PX4_GZ_STANDALONE=1; ',
                'else unset PX4_GZ_STANDALONE; fi && ',
                'export PX4_SIM_HOST_ADDR=', qgc_host_ip, ' && ',
                'export PX4_SIM_UDP_PORT=14550 && ',
                'export PX4_GCS_HOST_ADDR=', qgc_host_ip, ' && ',
                'export PX4_GCS_UDP_PORT=', qgc_udp_port, ' && ',
                'if [ "', headless, '" = "1" ] || [ "', headless, '" = "true" ]; then ',
                'export HEADLESS=1; ',
                'else unset HEADLESS; fi && ',
                'make px4_sitl ', px4_make_target,
            ],
        ],
        cwd=px4_autopilot_path,
        condition=IfCondition(start_px4),
        output='screen'
    )
    
    # Spawn the first cylinder (front, full height)
    spawn_cylinder_front = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(gz_model_path, 'cylinder', 'model.sdf'),
            '-name', 'cylinder_front',
            '-x', '5',     # 5 meters in front of the drone
            '-y', '0',     # centered on y-axis
            '-z', '0',     # at ground level
            '-R', '0',     # no roll
            '-P', '0',     # no pitch
            '-Y', '0',     # no yaw
            '-scale', '1 1 1',  # normal scale
            '-static'      # ensure it's static
        ],
        output='screen'
    )

    # Spawn the second cylinder (behind, 7m height)
    spawn_cylinder_back = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(gz_model_path, 'cylinder_short', 'model.sdf'),
            '-name', 'cylinder_back',
            '-x', '-5',    # 5 meters behind the drone
            '-y', '0',     # centered on y-axis
            '-z', '0',     # at ground level
            '-R', '0',     # no roll
            '-P', '0',     # no pitch
            '-Y', '0',     # no yaw
            '-static'      # ensure it's static
        ],
        output='screen'
    )

    # Bridge node for camera and odometry
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge',
        parameters=[{
            'use_sim_time': True,
        }],
        arguments=[
            # Front RGB Camera
            '/rgb_camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/rgb_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            
            # Front Depth Camera
            '/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image',
            # '/depth_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            
            # Down Mono Camera
            '/mono_camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/mono_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            
            # Downward Lidar
            '/rangefinder@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            
            # Clock and Odometry
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            ['/model/', model_instance, '/odometry_with_covariance@nav_msgs/msg/Odometry@gz.msgs.OdometryWithCovariance'],
        ],
        remappings=[
            # Front RGB Camera remappings
            ('/rgb_camera', '/drone/front_rgb'),
            ('/rgb_camera/camera_info', '/drone/front_rgb/camera_info'),
            
            # Front Depth Camera remappings
            ('/depth_camera', '/drone/front_depth'),
            # ('/depth_camera/depth_image', '/drone/front_depth/depth'),
            ('/depth_camera/points', '/drone/front_depth/points'),
            ('/camera_info', '/drone/front_depth/camera_info'),
            
            # Down Mono Camera remappings
            ('/mono_camera', '/drone/down_mono'),
            ('/mono_camera/camera_info', '/drone/down_mono/camera_info'),
            
            # Lidar remapping
            ('/rangefinder', '/drone/down_rangefinder'),
            
            # Gazebo ground-truth odometry for evaluation. Keep it separate
            # from PX4's /fmu/out/vehicle_odometry to avoid mixed message types.
            ((['/model/', model_instance, '/odometry_with_covariance']), '/sim/ground_truth/vehicle_odometry'),
        ],
        output='screen'
    )

    rangefinder_mission = Node(
        package='terrain_mapping_drone_control',
        executable='rangefinder_terrain_mission',
        name='rangefinder_terrain_mission',
        parameters=[{
            'use_sim_time': False,
            'mission_mode': mission_mode,
            'hover_relative_height': hover_relative_height,
            'require_preflight_checks': require_preflight_checks,
            'rvio_test_agl': rvio_test_agl,
            'rvio_test_climb_height': rvio_test_climb_height,
            'rvio_test_motion_profile': ParameterValue(
                rvio_test_motion_profile, value_type=str
            ),
            'rvio_test_axis': ParameterValue(rvio_test_axis, value_type=str),
            'rvio_test_axis_distance': rvio_test_axis_distance,
            'rvio_test_axis_speed': rvio_test_axis_speed,
        }],
        output='screen'
    )

    rvio_poc = Node(
        package='terrain_mapping_drone_control',
        executable='rvio_poc_node',
        name='rvio_poc_node',
        parameters=[{
            'use_sim_time': True,
            'image_topic': '/drone/down_mono',
            'camera_info_topic': '/drone/down_mono/camera_info',
            'range_topic': '/drone/down_rangefinder',
            'px4_sensor_combined_topic': '/fmu/out/sensor_combined',
            'px4_vehicle_attitude_topic': '/fmu/out/vehicle_attitude',
            'ground_truth_topic': '/sim/ground_truth/vehicle_odometry',
            'ground_truth_error_z_mode': 'ignore',
            'estimator_mode': rvio_estimator_mode,
            'mission_state_topic': '/mission/state',
            'start_on_mission_state': rvio_start_state,
            'flow_x_sign': rvio_flow_x_sign,
            'flow_y_sign': rvio_flow_y_sign,
            'camera_mount_roll_rad': rvio_camera_mount_roll_rad,
            'camera_mount_pitch_rad': rvio_camera_mount_pitch_rad,
            'camera_mount_yaw_rad': rvio_camera_mount_yaw_rad,
            'range_feature_use_attitude_projection': rvio_range_feature_use_attitude_projection,
            'range_feature_lidar_projection_radius_px': rvio_range_feature_lidar_projection_radius_px,
            'range_feature_reprojection_gate_px': rvio_range_feature_reprojection_gate_px,
            'camera_body_pitch_rad': rvio_camera_body_pitch_rad,
            'lidar_body_pitch_rad': rvio_lidar_body_pitch_rad,
            'lidar_body_roll_rad': rvio_lidar_body_roll_rad,
            'max_visual_velocity_mps': rvio_max_visual_velocity_mps,
            'max_range_feature_velocity_mps': rvio_max_range_feature_velocity_mps,
            'max_range_feature_pixel_rate_pxps': rvio_max_range_feature_pixel_rate_pxps,
        }],
        condition=IfCondition(PythonExpression([
            "'", start_rvio, "' in ['1', 'true', 'True'] and '",
            rvio_backend, "' == 'python'"
        ])),
        output='screen'
    )

    openvins_imu_bridge = Node(
        package='terrain_mapping_drone_control',
        executable='px4_imu_bridge',
        name='px4_imu_bridge',
        parameters=[{
            'use_sim_time': True,
            'px4_sensor_combined_topic': '/fmu/out/sensor_combined',
            'imu_topic': '/openvins/imu',
            'frame_id': 'imu',
        }],
        condition=IfCondition(PythonExpression([
            "'", start_rvio, "' in ['1', 'true', 'True'] and '",
            rvio_backend, "' == 'openvins'"
        ])),
        output='screen'
    )

    openvins_rvio_launcher = Node(
        package='terrain_mapping_drone_control',
        executable='openvins_on_state_launcher',
        name='openvins_on_state_launcher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'mission_state_topic': '/mission/state',
            'start_on_state': rvio_start_state,
            'config_path': os.path.join(
                pkg_share, 'config', 'openvins_mars_estimator.yaml'
            ),
            'verbosity': 'INFO',
            'topic_imu': '/openvins/imu',
            'topic_camera0': '/drone/down_mono',
            'topic_range': '/drone/down_rangefinder',
            'use_stereo': False,
            'max_cameras': 1,
            'range_feature_enable': True,
        }],
        condition=IfCondition(PythonExpression([
            "'", start_rvio, "' in ['1', 'true', 'True'] and '",
            rvio_backend, "' == 'openvins'"
        ])),
    )

    rvio_evaluator = Node(
        package='terrain_mapping_drone_control',
        executable='rvio_evaluator_node',
        name='rvio_evaluator_node',
        parameters=[{
            'use_sim_time': True,
            'rvio_odom_topic': '/rvio/odometry',
            'ground_truth_topic': '/sim/ground_truth/vehicle_odometry',
            'range_topic': '/drone/down_rangefinder',
            'write_csv': write_eval_csv,
            'mission_state_topic': '/mission/state',
            'start_on_mission_state': rvio_start_state,
        }],
        condition=IfCondition(start_rvio_evaluator),
        output='screen'
    )

    rvio_rqt_plot_launcher = Node(
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
        condition=IfCondition(start_rvio_rqt_plots),
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_discovery_server',
            default_value='false',
            description='Use a Fast DDS discovery server instead of ROS_AUTOMATIC_DISCOVERY_RANGE'),
        DeclareLaunchArgument(
            'discovery_server',
            default_value='127.0.0.1:11811',
            description='Fast DDS discovery server address used when use_discovery_server is true'),
        DeclareLaunchArgument(
            'discovery_range',
            default_value='LOCALHOST',
            description='ROS_AUTOMATIC_DISCOVERY_RANGE used when use_discovery_server is false'),
        # By default, use LOCALHOST discovery so CLI tools in this VM can echo
        # topics without the broader ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET setup.
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
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'px4_autopilot_path',
            default_value=os.environ.get('HOME', '/home/' + os.environ.get('USER', 'user')) + '/PX4-Autopilot',
            description='Path to PX4-Autopilot directory'),
        DeclareLaunchArgument(
            'qgc_host_ip',
            default_value='192.168.0.92',
            description='Mac host IP running QGroundControl; PX4 sends MAVLink UDP 14550 here'),
        DeclareLaunchArgument(
            'qgc_udp_port',
            default_value='14550',
            description='QGroundControl UDP listening port on the Mac host'),
        DeclareLaunchArgument(
            'headless',
            default_value='0',
            description='Set to 1 to run Gazebo without the GUI'),
        DeclareLaunchArgument(
            'start_gazebo',
            default_value='false',
            description='Experimental: start Gazebo separately and have PX4 attach with PX4_GZ_STANDALONE=1'),
        DeclareLaunchArgument(
            'start_px4',
            default_value='true',
            description='Set to false when PX4/Gazebo is already running in another terminal'),
        DeclareLaunchArgument(
            'start_microxrce_agent',
            default_value='true',
            description='Start the Micro XRCE-DDS Agent used by PX4 /fmu ROS 2 topics'),
        DeclareLaunchArgument(
            'microxrce_agent_port',
            default_value='8888',
            description='UDP port for the Micro XRCE-DDS Agent'),
        DeclareLaunchArgument(
            'px4_model',
            default_value='x500_ingenuity_mars',
            description='Gazebo model name used by PX4'),
        DeclareLaunchArgument(
            'px4_world',
            default_value='mars_px4',
            description='Gazebo world name used by PX4'),
        DeclareLaunchArgument(
            'px4_make_target',
            default_value='gz_x500_ingenuity_mars_mars_px4',
            description='PX4 SITL make target for the selected model/world'),
        DeclareLaunchArgument(
            'px4_model_pose',
            default_value='0,0,50,0,0,0',
            description='Initial PX4 Gazebo model pose as x,y,z,roll,pitch,yaw; default starts 50 m above the terrain during startup handoff'),
        DeclareLaunchArgument(
            'model_instance',
            default_value='x500_ingenuity_mars_0',
            description='Gazebo model instance name for bridged ground-truth odometry'),
        DeclareLaunchArgument(
            'spawn_cylinders',
            default_value='false',
            description='Set true to spawn the original assignment cylinders'),
        DeclareLaunchArgument(
            'start_rvio',
            default_value='true',
            description='Start the RVIO proof-of-concept estimator'),
        DeclareLaunchArgument(
            'start_rvio_evaluator',
            default_value='true',
            description='Start the RVIO relative-frame accuracy evaluator'),
        DeclareLaunchArgument(
            'start_rvio_rqt_plots',
            default_value='true',
            description='Open RVIO rqt_plot windows automatically when rvio_start_state is reached'),
        DeclareLaunchArgument(
            'write_eval_csv',
            default_value='false',
            description='Also write RVIO evaluator metrics to ~/.ros/rvio_eval; rqt_plot topics are always published'),
        DeclareLaunchArgument(
            'rvio_estimator_mode',
            default_value='range_feature_3d',
            description='RVIO estimator mode, normally range_feature_3d for full 3D translational range-feature pose estimation'),
        DeclareLaunchArgument(
            'rvio_backend',
            default_value='openvins',
            description='RVIO backend: openvins for range-feature OpenVINS or python for the legacy proof-of-concept node'),
        DeclareLaunchArgument(
            'rvio_flow_x_sign',
            default_value='-1.0',
            description='Optical-flow u-axis sign used by RVIO metric conversion'),
        DeclareLaunchArgument(
            'rvio_flow_y_sign',
            default_value='-1.0',
            description='Optical-flow v-axis sign used by RVIO metric conversion'),
        DeclareLaunchArgument(
            'rvio_camera_mount_roll_rad',
            default_value='0.0',
            description='Camera mount roll used for optical-flow rotation compensation'),
        DeclareLaunchArgument(
            'rvio_camera_mount_pitch_rad',
            default_value='0.0',
            description='Camera mount pitch used for optical-flow rotation compensation'),
        DeclareLaunchArgument(
            'rvio_camera_mount_yaw_rad',
            default_value='0.0',
            description='Camera mount yaw used for optical-flow rotation compensation'),
        DeclareLaunchArgument(
            'rvio_range_feature_use_attitude_projection',
            default_value='true',
            description='Project the lidar ray into the camera image using camera/lidar body extrinsics before seeding range features'),
        DeclareLaunchArgument(
            'rvio_range_feature_lidar_projection_radius_px',
            default_value='24.0',
            description='Pixel radius around the projected lidar ray used for range-feature seeding'),
        DeclareLaunchArgument(
            'rvio_range_feature_reprojection_gate_px',
            default_value='8.0',
            description='Maximum range-feature reprojection residual in pixels allowed into the EKF update'),
        DeclareLaunchArgument(
            'rvio_camera_body_pitch_rad',
            default_value='1.5707963267948966',
            description='SDF camera-link pitch from body/base_link used for lidar-to-image projection'),
        DeclareLaunchArgument(
            'rvio_lidar_body_pitch_rad',
            default_value='1.5707963267948966',
            description='SDF rangefinder-link pitch from body/base_link used for lidar-to-image projection'),
        DeclareLaunchArgument(
            'rvio_lidar_body_roll_rad',
            default_value='3.141592653589793',
            description='SDF rangefinder sensor roll used for lidar ray direction'),
        DeclareLaunchArgument(
            'rvio_max_visual_velocity_mps',
            default_value='4.0',
            description='Direct-mode only: reject visual updates above this horizontal speed'),
        DeclareLaunchArgument(
            'rvio_max_range_feature_velocity_mps',
            default_value='4.0',
            description='Direct-mode only: reject individual range-feature flow tracks above this metric speed'),
        DeclareLaunchArgument(
            'rvio_max_range_feature_pixel_rate_pxps',
            default_value='1000.0',
            description='Reject individual range-feature flow tracks above this pixel rate'),
        DeclareLaunchArgument(
            'mission_mode',
            default_value='auto_hover',
            description='Mission behavior for rangefinder_terrain_mission: auto_hover, survey, or rvio_test'),
        DeclareLaunchArgument(
            'hover_relative_height',
            default_value='2.0',
            description='Auto-hover climb height above current spawn/hold point in meters'),
        DeclareLaunchArgument(
            'rvio_test_agl',
            default_value='5.0',
            description='Legacy terrain-relative flight height parameter; rvio_test uses rvio_test_climb_height for takeoff'),
        DeclareLaunchArgument(
            'rvio_test_climb_height',
            default_value='5.0',
            description='Relative climb in meters before mission_mode:=rvio_test starts RVIO metrics'),
        DeclareLaunchArgument(
            'rvio_test_motion_profile',
            default_value='single_axis',
            description='RVIO test path profile: single_axis or waypoints'),
        DeclareLaunchArgument(
            'rvio_test_axis',
            default_value='x',
            description='Axis for rvio_test_motion_profile:=single_axis: x or y'),
        DeclareLaunchArgument(
            'rvio_test_axis_distance',
            default_value='15.0',
            description='Signed meters to move along rvio_test_axis during the single-axis RVIO test'),
        DeclareLaunchArgument(
            'rvio_test_axis_speed',
            default_value='0.5',
            description='Meters per second for the moving single-axis RVIO target'),
        DeclareLaunchArgument(
            'rvio_start_state',
            default_value='SURVEY',
            description='Mission state that resets/starts RVIO and metrics; empty starts immediately'),
        DeclareLaunchArgument(
            'require_preflight_checks',
            default_value='true',
            description='Wait for PX4 preflight checks before sending arm commands'),
        gazebo,
        microxrce_agent,
        TimerAction(
            period=4.0,
            actions=[px4_sitl],
            condition=IfCondition(start_px4),
        ),
        TimerAction(
            period=2.0,
            actions=[spawn_cylinder_front],
            condition=IfCondition(spawn_cylinders),
        ),
        TimerAction(
            period=2.5,
            actions=[spawn_cylinder_back],
            condition=IfCondition(spawn_cylinders),
        ),
        TimerAction(
            period=3.0,
            actions=[bridge]
        ),
        TimerAction(
            period=5.0,
            actions=[rangefinder_mission]
        ),
        TimerAction(
            period=5.0,
            actions=[rvio_poc],
            condition=IfCondition(start_rvio),
        ),
        TimerAction(
            period=5.0,
            actions=[openvins_imu_bridge, openvins_rvio_launcher],
            condition=IfCondition(start_rvio),
        ),
        TimerAction(
            period=6.0,
            actions=[rvio_evaluator],
            condition=IfCondition(start_rvio_evaluator),
        ),
        TimerAction(
            period=6.0,
            actions=[rvio_rqt_plot_launcher],
            condition=IfCondition(start_rvio_rqt_plots),
        )
    ])
