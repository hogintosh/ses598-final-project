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
    start_rvio = LaunchConfiguration('start_rvio')
    start_rvio_evaluator = LaunchConfiguration('start_rvio_evaluator')
    start_rvio_rqt_plots = LaunchConfiguration('start_rvio_rqt_plots')
    write_eval_csv = LaunchConfiguration('write_eval_csv')
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
            'rvio_backend',
            default_value='openvins',
            description='RVIO backend. The pruned project keeps openvins as the supported backend.'),
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
            period=3.0,
            actions=[bridge]
        ),
        TimerAction(
            period=5.0,
            actions=[rangefinder_mission]
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
