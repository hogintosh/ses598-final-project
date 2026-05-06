#!/usr/bin/env python3

import math
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry, Path
from px4_msgs.msg import SensorCombined, VehicleAttitude
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
from std_msgs.msg import Float32, Int32, String


def yaw_to_quaternion(yaw):
    half_yaw = 0.5 * yaw
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


def euler_to_quaternion(roll, pitch, yaw):
    cr = math.cos(0.5 * float(roll))
    sr = math.sin(0.5 * float(roll))
    cp = math.cos(0.5 * float(pitch))
    sp = math.sin(0.5 * float(pitch))
    cy = math.cos(0.5 * float(yaw))
    sy = math.sin(0.5 * float(yaw))
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ], dtype=float)


def quaternion_to_rotation_matrix(q):
    w, x, y, z = [float(v) for v in q]
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 1e-9:
        return np.eye(3, dtype=float)
    w /= norm
    x /= norm
    y /= norm
    z /= norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=float)


def euler_to_rotation_matrix(roll, pitch, yaw):
    cr = math.cos(float(roll))
    sr = math.sin(float(roll))
    cp = math.cos(float(pitch))
    sp = math.sin(float(pitch))
    cy = math.cos(float(yaw))
    sy = math.sin(float(yaw))
    rx = np.array([
        [1.0, 0.0, 0.0],
        [0.0, cr, -sr],
        [0.0, sr, cr],
    ], dtype=float)
    ry = np.array([
        [cp, 0.0, sp],
        [0.0, 1.0, 0.0],
        [-sp, 0.0, cp],
    ], dtype=float)
    rz = np.array([
        [cy, -sy, 0.0],
        [sy, cy, 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=float)
    return rz @ ry @ rx


def vector_from_parameter(values, expected_len, default):
    try:
        parsed = [float(v) for v in values]
    except TypeError:
        parsed = list(default)
    if len(parsed) != expected_len:
        parsed = list(default)
    return np.asarray(parsed, dtype=float)


def quaternion_to_euler(q):
    w, x, y, z = [float(v) for v in q]
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 1e-9:
        return 0.0, 0.0, 0.0
    w /= norm
    x /= norm
    y /= norm
    z /= norm
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = 2.0 * (w * y - z * x)
    sin_pitch = max(-1.0, min(1.0, sin_pitch))
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def stamp_to_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_rotation_matrix(yaw):
    c = math.cos(float(yaw))
    s = math.sin(float(yaw))
    return np.array([
        [c, -s, 0.0],
        [s, c, 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=float)


class RvioPocNode(Node):
    """Range-visual-inertial odometry proof of concept for downward flight."""

    def __init__(self):
        super().__init__('rvio_poc_node')

        self.declare_parameter('image_topic', '/drone/down_mono')
        self.declare_parameter('camera_info_topic', '/drone/down_mono/camera_info')
        self.declare_parameter('range_topic', '/drone/down_rangefinder')
        self.declare_parameter('imu_topic', '/drone/imu')
        self.declare_parameter('px4_sensor_combined_topic', '/fmu/out/sensor_combined')
        self.declare_parameter('px4_vehicle_attitude_topic', '/fmu/out/vehicle_attitude')
        self.declare_parameter('ground_truth_topic', '/sim/ground_truth/vehicle_odometry')
        self.declare_parameter('odom_topic', '/rvio/odometry')
        self.declare_parameter('path_topic', '/rvio/path')
        self.declare_parameter('error_topic', '/rvio/ground_truth_error')
        self.declare_parameter('debug_image_topic', '/rvio/debug_image')
        self.declare_parameter('ground_truth_error_z_mode', 'ignore')
        self.declare_parameter('estimator_mode', 'range_feature_3d')
        self.declare_parameter('mission_state_topic', '/mission/state')
        self.declare_parameter('start_on_mission_state', '')
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('body_frame', 'rvio/base_link')
        self.declare_parameter('max_features', 250)
        self.declare_parameter('quality_level', 0.01)
        self.declare_parameter('min_distance_px', 10.0)
        self.declare_parameter('min_tracked_features', 12)
        self.declare_parameter('max_flow_px', 80.0)
        self.declare_parameter('flow_reject_mad', 3.5)
        self.declare_parameter('range_filter_alpha', 0.35)
        self.declare_parameter('velocity_filter_alpha', 0.55)
        self.declare_parameter('accel_weight', 1.0)
        self.declare_parameter('visual_weight', 0.75)
        self.declare_parameter('ekf_process_accel_std', 1.5)
        self.declare_parameter('ekf_initial_position_std', 1.0)
        self.declare_parameter('ekf_initial_velocity_std', 1.0)
        self.declare_parameter('ekf_initial_attitude_std', 0.10)
        self.declare_parameter('ekf_initial_accel_bias_std', 0.20)
        self.declare_parameter('ekf_initial_gyro_bias_std', 0.02)
        self.declare_parameter('ekf_accel_bias_rw_std', 0.02)
        self.declare_parameter('ekf_gyro_bias_rw_std', 0.002)
        self.declare_parameter('ekf_process_gyro_std', 0.02)
        self.declare_parameter('use_px4_attitude_measurement', True)
        self.declare_parameter('attitude_measurement_std', 0.035)
        self.declare_parameter('range_z_std', 0.05)
        self.declare_parameter('range_feature_depth_std', 0.25)
        self.declare_parameter('range_feature_inv_depth_std_floor', 1.0e-4)
        self.declare_parameter('range_feature_flow_rate_std_px', 1.5)
        self.declare_parameter('range_feature_reprojection_gate_px', 8.0)
        self.declare_parameter('require_range_feature_update', True)
        self.declare_parameter('compensate_camera_rotation', True)
        self.declare_parameter('max_visual_velocity_mps', 4.0)
        self.declare_parameter('max_range_feature_velocity_mps', 4.0)
        self.declare_parameter('max_range_feature_pixel_rate_pxps', 1000.0)
        self.declare_parameter('camera_mount_roll_rad', 0.0)
        self.declare_parameter('camera_mount_pitch_rad', 0.0)
        self.declare_parameter('camera_mount_yaw_rad', 0.0)
        self.declare_parameter('camera_body_translation_m', [0.0, 0.0, 0.15])
        self.declare_parameter('lidar_body_translation_m', [0.0, 0.0, -0.05])
        self.declare_parameter('camera_body_roll_rad', 0.0)
        self.declare_parameter('camera_body_pitch_rad', 1.5707963267948966)
        self.declare_parameter('camera_body_yaw_rad', 0.0)
        self.declare_parameter('lidar_body_roll_rad', 3.141592653589793)
        self.declare_parameter('lidar_body_pitch_rad', 1.5707963267948966)
        self.declare_parameter('lidar_body_yaw_rad', 0.0)
        self.declare_parameter('range_feature_lidar_projection_radius_px', 24.0)
        self.declare_parameter('range_feature_use_attitude_projection', True)
        self.declare_parameter('flow_x_sign', -1.0)
        self.declare_parameter('flow_y_sign', -1.0)
        self.declare_parameter('center_corner_window', 9)
        self.declare_parameter('range_feature_min_score', 5.0e-6)
        self.declare_parameter('range_feature_peak_lookahead', 0)
        self.declare_parameter('range_feature_min_separation_sec', 0.15)
        self.declare_parameter('range_feature_search_radius_px', 18.0)
        self.declare_parameter('range_feature_max_center_offset_px', 16.0)
        self.declare_parameter('range_feature_min_existing_separation_px', 8.0)
        self.declare_parameter('range_feature_offset_depth_uncertainty_scale', 1.5)
        self.declare_parameter('max_range_features', 30)
        self.declare_parameter('min_range_feature_tracks_for_update', 1)
        self.declare_parameter('range_feature_max_age_frames', 120)
        self.declare_parameter('range_feature_min_depth_m', 0.1)
        self.declare_parameter('range_feature_max_depth_m', 100.0)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('path_max_length', 1000)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        self.range_topic = str(self.get_parameter('range_topic').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.px4_sensor_combined_topic = str(
            self.get_parameter('px4_sensor_combined_topic').value
        )
        self.px4_vehicle_attitude_topic = str(
            self.get_parameter('px4_vehicle_attitude_topic').value
        )
        self.ground_truth_topic = str(self.get_parameter('ground_truth_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.path_topic = str(self.get_parameter('path_topic').value)
        self.error_topic = str(self.get_parameter('error_topic').value)
        self.debug_image_topic = str(self.get_parameter('debug_image_topic').value)
        self.ground_truth_error_z_mode = str(
            self.get_parameter('ground_truth_error_z_mode').value
        ).lower()
        self.estimator_mode = str(self.get_parameter('estimator_mode').value).lower()
        if self.estimator_mode == 'ekf_lite':
            self.get_logger().warn(
                'estimator_mode:=ekf_lite is deprecated; using range_feature_3d.'
            )
            self.estimator_mode = 'range_feature_3d'
        self.use_range_feature_3d = self.estimator_mode in (
            'range_feature_3d',
            'full_pose',
        )
        self.mission_state_topic = str(
            self.get_parameter('mission_state_topic').value
        )
        self.start_on_mission_state = str(
            self.get_parameter('start_on_mission_state').value
        ).upper()
        self.world_frame = str(self.get_parameter('world_frame').value)
        self.body_frame = str(self.get_parameter('body_frame').value)
        self.max_features = int(self.get_parameter('max_features').value)
        self.quality_level = float(self.get_parameter('quality_level').value)
        self.min_distance_px = float(self.get_parameter('min_distance_px').value)
        self.min_tracked_features = int(self.get_parameter('min_tracked_features').value)
        self.max_flow_px = float(self.get_parameter('max_flow_px').value)
        self.flow_reject_mad = float(self.get_parameter('flow_reject_mad').value)
        self.range_filter_alpha = float(self.get_parameter('range_filter_alpha').value)
        self.velocity_filter_alpha = float(
            self.get_parameter('velocity_filter_alpha').value
        )
        self.accel_weight = float(self.get_parameter('accel_weight').value)
        self.visual_weight = float(self.get_parameter('visual_weight').value)
        self.ekf_process_accel_std = float(
            self.get_parameter('ekf_process_accel_std').value
        )
        self.ekf_initial_position_std = float(
            self.get_parameter('ekf_initial_position_std').value
        )
        self.ekf_initial_velocity_std = float(
            self.get_parameter('ekf_initial_velocity_std').value
        )
        self.ekf_initial_attitude_std = float(
            self.get_parameter('ekf_initial_attitude_std').value
        )
        self.ekf_initial_accel_bias_std = float(
            self.get_parameter('ekf_initial_accel_bias_std').value
        )
        self.ekf_initial_gyro_bias_std = float(
            self.get_parameter('ekf_initial_gyro_bias_std').value
        )
        self.ekf_accel_bias_rw_std = float(
            self.get_parameter('ekf_accel_bias_rw_std').value
        )
        self.ekf_gyro_bias_rw_std = float(
            self.get_parameter('ekf_gyro_bias_rw_std').value
        )
        self.ekf_process_gyro_std = float(
            self.get_parameter('ekf_process_gyro_std').value
        )
        self.use_px4_attitude_measurement = bool(
            self.get_parameter('use_px4_attitude_measurement').value
        )
        self.attitude_measurement_std = float(
            self.get_parameter('attitude_measurement_std').value
        )
        self.range_z_std = float(self.get_parameter('range_z_std').value)
        self.range_feature_depth_std = float(
            self.get_parameter('range_feature_depth_std').value
        )
        self.range_feature_inv_depth_std_floor = float(
            self.get_parameter('range_feature_inv_depth_std_floor').value
        )
        self.range_feature_flow_rate_std_px = float(
            self.get_parameter('range_feature_flow_rate_std_px').value
        )
        self.range_feature_reprojection_gate_px = float(
            self.get_parameter('range_feature_reprojection_gate_px').value
        )
        self.require_range_feature_update = bool(
            self.get_parameter('require_range_feature_update').value
        )
        self.compensate_camera_rotation = bool(
            self.get_parameter('compensate_camera_rotation').value
        )
        self.max_visual_velocity_mps = float(
            self.get_parameter('max_visual_velocity_mps').value
        )
        self.max_range_feature_velocity_mps = float(
            self.get_parameter('max_range_feature_velocity_mps').value
        )
        self.max_range_feature_pixel_rate_pxps = float(
            self.get_parameter('max_range_feature_pixel_rate_pxps').value
        )
        camera_mount_roll_rad = float(
            self.get_parameter('camera_mount_roll_rad').value
        )
        camera_mount_pitch_rad = float(
            self.get_parameter('camera_mount_pitch_rad').value
        )
        camera_mount_yaw_rad = float(
            self.get_parameter('camera_mount_yaw_rad').value
        )
        self.camera_mount_rotation = euler_to_rotation_matrix(
            camera_mount_roll_rad,
            camera_mount_pitch_rad,
            camera_mount_yaw_rad,
        )
        self.camera_body_translation = vector_from_parameter(
            self.get_parameter('camera_body_translation_m').value,
            3,
            [0.0, 0.0, 0.15],
        )
        self.lidar_body_translation = vector_from_parameter(
            self.get_parameter('lidar_body_translation_m').value,
            3,
            [0.0, 0.0, -0.05],
        )
        self.camera_body_rotation = euler_to_rotation_matrix(
            float(self.get_parameter('camera_body_roll_rad').value),
            float(self.get_parameter('camera_body_pitch_rad').value),
            float(self.get_parameter('camera_body_yaw_rad').value),
        )
        self.lidar_body_rotation = euler_to_rotation_matrix(
            float(self.get_parameter('lidar_body_roll_rad').value),
            float(self.get_parameter('lidar_body_pitch_rad').value),
            float(self.get_parameter('lidar_body_yaw_rad').value),
        )
        self.body_camera_rotation = self._body_from_cv_camera_rotation(
            self.camera_body_rotation
        )
        self.camera_body_rotation_cv = self.body_camera_rotation.T
        self.body_lidar_ray = self._unit_vector(
            self.lidar_body_rotation @ np.array([1.0, 0.0, 0.0], dtype=float)
        )
        self.range_feature_lidar_projection_radius_px = float(
            self.get_parameter('range_feature_lidar_projection_radius_px').value
        )
        self.range_feature_use_attitude_projection = bool(
            self.get_parameter('range_feature_use_attitude_projection').value
        )
        self.flow_x_sign = float(self.get_parameter('flow_x_sign').value)
        self.flow_y_sign = float(self.get_parameter('flow_y_sign').value)
        self.center_corner_window = int(
            self.get_parameter('center_corner_window').value
        )
        self.range_feature_min_score = float(
            self.get_parameter('range_feature_min_score').value
        )
        self.range_feature_peak_lookahead = int(
            self.get_parameter('range_feature_peak_lookahead').value
        )
        self.range_feature_min_separation_sec = float(
            self.get_parameter('range_feature_min_separation_sec').value
        )
        self.range_feature_search_radius_px = float(
            self.get_parameter('range_feature_search_radius_px').value
        )
        self.range_feature_max_center_offset_px = float(
            self.get_parameter('range_feature_max_center_offset_px').value
        )
        self.range_feature_min_existing_separation_px = float(
            self.get_parameter('range_feature_min_existing_separation_px').value
        )
        self.range_feature_offset_depth_uncertainty_scale = float(
            self.get_parameter('range_feature_offset_depth_uncertainty_scale').value
        )
        self.max_range_features = int(self.get_parameter('max_range_features').value)
        self.min_range_feature_tracks_for_update = int(
            self.get_parameter('min_range_feature_tracks_for_update').value
        )
        self.range_feature_max_age_frames = int(
            self.get_parameter('range_feature_max_age_frames').value
        )
        self.range_feature_min_depth_m = float(
            self.get_parameter('range_feature_min_depth_m').value
        )
        self.range_feature_max_depth_m = float(
            self.get_parameter('range_feature_max_depth_m').value
        )
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self.path_max_length = int(self.get_parameter('path_max_length').value)

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.bridge = CvBridge()
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.range_m = None
        self.last_image_time = None
        self.last_imu_time = None
        self.attitude_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        self.prev_attitude_quat = None
        self.prev_image_attitude_quat = None
        self.roll = 0.0
        self.pitch = 0.0
        self.prev_gray = None
        self.prev_points = None
        self.prev_stamp = None
        self.ground_truth = None
        self.center_score_history = deque(
            maxlen=max(3, self.range_feature_peak_lookahead + 2)
        )
        self.active_range_features = []
        self.next_range_feature_id = 1
        self.last_range_feature_time = None

        # Local RVIO pose estimate. Attitude is currently supplied by PX4, while
        # translation is estimated in the EKF below.
        self.position = np.zeros(3, dtype=float)
        self.velocity = np.zeros(3, dtype=float)
        # Base EKF state: px, py, pz, vx, vy, vz, roll, pitch, yaw,
        # accel_bias_x/y/z, gyro_bias_x/y/z.
        # Range-feature inverse depths are appended dynamically.
        self.base_state_size = 15
        self.ekf_state = np.zeros(self.base_state_size, dtype=float)
        self.ekf_cov = self._initial_ekf_covariance()
        self.latest_world_accel = np.zeros(3, dtype=float)
        self.yaw = 0.0
        self.path = deque(maxlen=self.path_max_length)
        self.frame_count = 0
        self.visual_update_count = 0
        self.estimator_active = self.start_on_mission_state == ''
        self.last_mission_state = ''

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.error_pub = self.create_publisher(Vector3Stamped, self.error_topic, 10)
        self.debug_image_pub = self.create_publisher(Image, self.debug_image_topic, 10)
        self.center_corner_score_pub = self.create_publisher(
            Float32, '/rvio/center_corner_score', 10
        )
        self.range_feature_count_pub = self.create_publisher(
            Int32, '/rvio/range_feature_count', 10
        )
        self.range_feature_depth_pub = self.create_publisher(
            Float32, '/rvio/latest_range_feature_depth', 10
        )
        self.range_feature_center_offset_pub = self.create_publisher(
            Float32, '/rvio/latest_range_feature_center_offset_px', 10
        )
        self.visual_update_source_pub = self.create_publisher(
            String, '/rvio/visual_update_source', 10
        )
        self.debug_flow_u_pub = self.create_publisher(
            Float32, '/rvio_debug/median_flow_u_px', 10
        )
        self.debug_image_dt_pub = self.create_publisher(
            Float32, '/rvio_debug/image_dt_sec', 10
        )
        self.debug_flow_v_pub = self.create_publisher(
            Float32, '/rvio_debug/median_flow_v_px', 10
        )
        self.debug_range_flow_u_pub = self.create_publisher(
            Float32, '/rvio_debug/range_flow_u_px', 10
        )
        self.debug_range_flow_v_pub = self.create_publisher(
            Float32, '/rvio_debug/range_flow_v_px', 10
        )
        self.debug_body_vx_pub = self.create_publisher(
            Float32, '/rvio_debug/body_vx_mps', 10
        )
        self.debug_body_vy_pub = self.create_publisher(
            Float32, '/rvio_debug/body_vy_mps', 10
        )
        self.debug_world_vx_pub = self.create_publisher(
            Float32, '/rvio_debug/world_vx_mps', 10
        )
        self.debug_world_vy_pub = self.create_publisher(
            Float32, '/rvio_debug/world_vy_mps', 10
        )
        self.debug_gt_vx_pub = self.create_publisher(
            Float32, '/rvio_debug/ground_truth_vx_mps', 10
        )
        self.debug_gt_vy_pub = self.create_publisher(
            Float32, '/rvio_debug/ground_truth_vy_mps', 10
        )
        self.debug_velocity_sign_score_pub = self.create_publisher(
            Float32, '/rvio_debug/velocity_sign_score', 10
        )
        self.debug_accepted_range_feature_count_pub = self.create_publisher(
            Int32, '/rvio_debug/accepted_range_feature_count', 10
        )
        self.debug_rejected_range_feature_count_pub = self.create_publisher(
            Int32, '/rvio_debug/rejected_range_feature_count', 10
        )
        self.debug_candidate_range_feature_pixel_rate_pub = self.create_publisher(
            Float32, '/rvio_debug/candidate_range_feature_pixel_rate_pxps', 10
        )
        self.debug_candidate_range_feature_speed_pub = self.create_publisher(
            Float32, '/rvio_debug/candidate_range_feature_speed_mps', 10
        )
        self.debug_range_feature_reject_reason_pub = self.create_publisher(
            String, '/rvio_debug/range_feature_reject_reason', 10
        )
        self.debug_reprojection_residual_pub = self.create_publisher(
            Float32, '/rvio_debug/range_feature_reprojection_residual_px', 10
        )
        self.debug_lidar_projection_u_pub = self.create_publisher(
            Float32, '/rvio_debug/lidar_projection_u_px', 10
        )
        self.debug_lidar_projection_v_pub = self.create_publisher(
            Float32, '/rvio_debug/lidar_projection_v_px', 10
        )
        self.debug_lidar_feature_offset_pub = self.create_publisher(
            Float32, '/rvio_debug/lidar_feature_offset_px', 10
        )

        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_cb, 10)
        self.create_subscription(Image, self.image_topic, self.image_cb, sensor_qos)
        self.create_subscription(LaserScan, self.range_topic, self.range_cb, sensor_qos)
        self.create_subscription(Imu, self.imu_topic, self.imu_cb, sensor_qos)
        self.create_subscription(
            String, self.mission_state_topic, self.mission_state_cb, state_qos
        )
        self.create_subscription(
            SensorCombined,
            self.px4_sensor_combined_topic,
            self.sensor_combined_cb,
            px4_qos,
        )
        self.create_subscription(
            VehicleAttitude,
            self.px4_vehicle_attitude_topic,
            self.vehicle_attitude_cb,
            px4_qos,
        )
        self.create_subscription(
            Odometry, self.ground_truth_topic, self.ground_truth_cb, sensor_qos
        )

        if not self.estimator_active:
            self.get_logger().info(
                'RVIO estimator waiting for mission state %s on %s'
                % (self.start_on_mission_state, self.mission_state_topic)
            )
        self.get_logger().info(
            'RVIO POC ready: camera=%s range=%s imu=%s px4_imu=%s attitude=%s mode=%s -> %s'
            % (
                self.image_topic,
                self.range_topic,
                self.imu_topic,
                self.px4_sensor_combined_topic,
                self.px4_vehicle_attitude_topic,
                self.estimator_mode,
                self.odom_topic,
            )
        )

    def camera_info_cb(self, msg):
        if len(msg.k) >= 6:
            self.fx = float(msg.k[0])
            self.fy = float(msg.k[4])
            self.cx = float(msg.k[2])
            self.cy = float(msg.k[5])

    def mission_state_cb(self, msg):
        state = str(msg.data).upper()
        self.last_mission_state = state
        if self.estimator_active or state != self.start_on_mission_state:
            return

        self._reset_estimator_state()
        self.estimator_active = True
        self.get_logger().info(
            'RVIO estimator activated at mission state %s; local origin reset.'
            % state
        )

    def _initial_ekf_covariance(self):
        return np.diag([
            self.ekf_initial_position_std ** 2,
            self.ekf_initial_position_std ** 2,
            self.ekf_initial_position_std ** 2,
            self.ekf_initial_velocity_std ** 2,
            self.ekf_initial_velocity_std ** 2,
            self.ekf_initial_velocity_std ** 2,
            self.ekf_initial_attitude_std ** 2,
            self.ekf_initial_attitude_std ** 2,
            self.ekf_initial_attitude_std ** 2,
            self.ekf_initial_accel_bias_std ** 2,
            self.ekf_initial_accel_bias_std ** 2,
            self.ekf_initial_accel_bias_std ** 2,
            self.ekf_initial_gyro_bias_std ** 2,
            self.ekf_initial_gyro_bias_std ** 2,
            self.ekf_initial_gyro_bias_std ** 2,
        ]).astype(float)

    def _reset_estimator_state(self):
        z = self.range_m if self.range_m is not None else self.position[2]
        self.position[:] = 0.0
        self.velocity[:] = 0.0
        self.position[2] = z
        self.ekf_state = np.zeros(self.base_state_size, dtype=float)
        self.ekf_state[2] = z
        self.ekf_state[6:9] = np.array([self.roll, self.pitch, self.yaw], dtype=float)
        self.ekf_cov = self._initial_ekf_covariance()
        self.latest_world_accel[:] = 0.0
        self.yaw = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.prev_image_attitude_quat = None
        self.prev_gray = None
        self.prev_points = None
        self.prev_stamp = None
        self.active_range_features = []
        self.center_score_history.clear()
        self.last_range_feature_time = None
        self.path.clear()
        self.frame_count = 0
        self.visual_update_count = 0

    def range_cb(self, msg):
        valid_ranges = [
            float(r)
            for r in msg.ranges
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max
        ]
        if not valid_ranges:
            return

        measured = min(valid_ranges)
        if self.range_m is None:
            self.range_m = measured
        else:
            alpha = self.range_filter_alpha
            self.range_m = alpha * measured + (1.0 - alpha) * self.range_m

        if self.use_range_feature_3d and self.estimator_active:
            self._ekf_update_range_z(self.range_m)
            self._sync_state_from_ekf()
        else:
            self.position[2] = self.range_m

    def vehicle_attitude_cb(self, msg):
        quat = np.asarray(msg.q, dtype=float)
        if len(quat) != 4 or not np.all(np.isfinite(quat)):
            return
        measured_roll, measured_pitch, measured_yaw = quaternion_to_euler(quat)
        if self.use_range_feature_3d and self.estimator_active:
            if self.use_px4_attitude_measurement:
                self._ekf_update_attitude_measurement(
                    measured_roll,
                    measured_pitch,
                    measured_yaw,
                )
                self._sync_state_from_ekf()
            return
        self.prev_attitude_quat = self.attitude_quat.copy()
        self.attitude_quat = quat
        self.roll = measured_roll
        self.pitch = measured_pitch
        self.yaw = measured_yaw

    def imu_cb(self, msg):
        now = stamp_to_seconds(msg.header.stamp)
        if self.last_imu_time is None:
            self.last_imu_time = now
            return

        dt = max(0.0, min(0.05, now - self.last_imu_time))
        self.last_imu_time = now
        self._propagate_imu(
            dt,
            np.array([
                float(msg.angular_velocity.x),
                float(msg.angular_velocity.y),
                float(msg.angular_velocity.z),
            ], dtype=float),
            float(msg.linear_acceleration.x),
            float(msg.linear_acceleration.y),
            float(msg.linear_acceleration.z),
        )

    def sensor_combined_cb(self, msg):
        now = float(msg.timestamp) * 1e-6
        if self.last_imu_time is None:
            self.last_imu_time = now
            return

        dt = max(0.0, min(0.05, now - self.last_imu_time))
        self.last_imu_time = now
        self._propagate_imu(
            dt,
            np.array([
                float(msg.gyro_rad[0]),
                float(msg.gyro_rad[1]),
                float(msg.gyro_rad[2]),
            ], dtype=float),
            float(msg.accelerometer_m_s2[0]),
            float(msg.accelerometer_m_s2[1]),
            float(msg.accelerometer_m_s2[2]),
        )

    def _propagate_imu(self, dt, gyro_rad, accel_x, accel_y, accel_z=0.0):
        if dt <= 0.0:
            return

        if not self.estimator_active:
            return

        gyro_rad = np.asarray(gyro_rad, dtype=float).reshape(3)
        if self.prev_attitude_quat is None:
            self.yaw = wrap_angle(self.yaw + gyro_rad[2] * dt)
        if self.use_range_feature_3d:
            self._ekf_predict(
                dt,
                np.array([accel_x, accel_y, accel_z], dtype=float),
                gyro_rad,
            )
            self._sync_state_from_ekf()
            return

        accel_world = self._world_accel_without_gravity(accel_x, accel_y, accel_z)
        self.latest_world_accel = accel_world.copy()
        self.velocity[:2] += self.accel_weight * accel_world[:2] * dt
        self.position[:2] += self.velocity[:2] * dt

    def _world_accel_without_gravity(self, accel_x, accel_y, accel_z):
        body_accel = np.array([accel_x, accel_y, accel_z], dtype=float)
        if np.all(np.isfinite(body_accel)) and abs(accel_z) > 1.0:
            world_accel = self._estimated_world_from_body_rotation() @ body_accel
            # Remove the dominant gravity component from whichever world Z sign
            # the simulator/bridge uses. This keeps pitch/roll from masquerading
            # as horizontal translation during the EKF prediction.
            if abs(world_accel[2]) > 5.0:
                world_accel[2] -= math.copysign(9.80665, world_accel[2])
            return world_accel

        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        return np.array(
            [c * accel_x - s * accel_y, s * accel_x + c * accel_y, 0.0],
            dtype=float,
        )

    def _estimated_world_from_body_rotation(self, state=None):
        if state is None:
            if self.use_range_feature_3d and len(self.ekf_state) >= 9:
                roll, pitch, yaw = self.ekf_state[6:9]
            else:
                roll, pitch, yaw = self.roll, self.pitch, self.yaw
        else:
            roll, pitch, yaw = np.asarray(state, dtype=float).reshape(-1)[6:9]
        return euler_to_rotation_matrix(roll, pitch, yaw)

    def _euler_rates_from_body_gyro(self, rpy, gyro_body):
        roll, pitch, _ = [float(v) for v in rpy]
        gx, gy, gz = [float(v) for v in gyro_body]
        sr = math.sin(roll)
        cr = math.cos(roll)
        cp = math.cos(pitch)
        cp_safe = math.copysign(max(abs(cp), 1.0e-3), cp if abs(cp) > 1.0e-9 else 1.0)
        tp = math.sin(pitch) / cp_safe
        return np.array([
            gx + sr * tp * gy + cr * tp * gz,
            cr * gy - sr * gz,
            (sr / cp_safe) * gy + (cr / cp_safe) * gz,
        ], dtype=float)

    def image_cb(self, msg):
        if self.fx is None or self.fy is None or self.range_m is None:
            return

        if not self.estimator_active:
            self.prev_gray = None
            self.prev_points = None
            self.prev_stamp = None
            return

        try:
            gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as exc:
            self.get_logger().warn(f'Could not convert RVIO image: {exc}')
            return

        stamp = stamp_to_seconds(msg.header.stamp)
        if stamp <= 0.0:
            stamp = self.get_clock().now().nanoseconds * 1e-9

        center_score = self._center_corner_score(gray)
        self._publish_float(self.center_corner_score_pub, center_score)
        self._maybe_seed_range_feature(gray, stamp, center_score)

        if self.prev_gray is None:
            self._reset_tracks(gray, stamp)
            return

        dt = stamp - self.prev_stamp
        if dt <= 1e-4 or dt > 0.5:
            self._reset_tracks(gray, stamp)
            return
        self._publish_float(self.debug_image_dt_pub, dt)

        range_feature_flows = self._track_range_features(
            gray, self.attitude_quat, stamp
        )
        accepted_range_feature_flows = self._filter_range_feature_flows(
            range_feature_flows, dt
        )

        update_source = 'waiting_for_range_feature'
        ekf_visual_update = False
        if self.use_range_feature_3d:
            if len(accepted_range_feature_flows) >= self.min_range_feature_tracks_for_update:
                ekf_visual_update = self._ekf_update_range_feature_reprojection(
                    accepted_range_feature_flows
                )
                self._sync_state_from_ekf()
                update_source = (
                    'range_feature_reprojection'
                    if ekf_visual_update
                    else 'rejected_range_feature_geometry'
                )
                if ekf_visual_update:
                    self.visual_update_count += 1
            elif range_feature_flows:
                update_source = 'rejected_range_feature_track'
            self._publish_visual_update_source(update_source)
            self._publish_range_feature_count()
            self._publish_estimate(msg.header.stamp)

        if self.prev_points is None or len(self.prev_points) < self.min_tracked_features:
            self._reset_tracks(gray, stamp)
            return

        next_points, status, _ = cv2.calcOpticalFlowPyrLK(
            self.prev_gray,
            gray,
            self.prev_points,
            None,
            winSize=(21, 21),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
        )
        if next_points is None or status is None:
            self._reset_tracks(gray, stamp)
            return

        tracked_prev = self.prev_points[status.reshape(-1) == 1].reshape(-1, 2)
        tracked_next = next_points[status.reshape(-1) == 1].reshape(-1, 2)
        if len(tracked_next) < self.min_tracked_features:
            self._reset_tracks(gray, stamp)
            return

        flow = tracked_next - tracked_prev
        flow_norm = np.linalg.norm(flow, axis=1)
        bounded = flow_norm < self.max_flow_px
        flow = flow[bounded]
        tracked_next = tracked_next[bounded]
        if len(flow) < self.min_tracked_features:
            self._reset_tracks(gray, stamp)
            return

        median_flow = np.median(flow, axis=0)
        residual = np.linalg.norm(flow - median_flow, axis=1)
        mad = np.median(np.abs(residual - np.median(residual)))
        if mad > 1e-6:
            inliers = residual < self.flow_reject_mad * 1.4826 * mad
            flow = flow[inliers]
            tracked_next = tracked_next[inliers]
        if len(flow) < self.min_tracked_features:
            self._reset_tracks(gray, stamp)
            return

        if self.use_range_feature_3d:
            debug_body_delta, _ = self._metric_body_delta_from_flow(
                flow, accepted_range_feature_flows, len(range_feature_flows)
            )
            if debug_body_delta is not None:
                debug_body_delta = np.asarray(debug_body_delta, dtype=float)
                c_yaw = math.cos(self.yaw)
                s_yaw = math.sin(self.yaw)
                debug_world_delta = np.array(
                    [
                        c_yaw * debug_body_delta[0] - s_yaw * debug_body_delta[1],
                        s_yaw * debug_body_delta[0] + c_yaw * debug_body_delta[1],
                    ],
                    dtype=float,
                )
                self._publish_flow_calibration_debug(
                    flow,
                    range_feature_flows,
                    debug_body_delta,
                    debug_world_delta,
                    debug_world_delta / dt,
                    dt,
                )
            if self.publish_debug_image:
                self._publish_debug_image(
                    gray,
                    tracked_next,
                    flow,
                    msg.header.stamp,
                    accepted_range_feature_flows,
                    update_source,
                )
            self.prev_gray = gray
            self.prev_points = tracked_next.reshape(-1, 1, 2).astype(np.float32)
            self.prev_stamp = stamp
            self.prev_image_attitude_quat = self.attitude_quat.copy()
            self.frame_count += 1
            return

        body_delta, update_source = self._metric_body_delta_from_flow(
            flow, accepted_range_feature_flows, len(range_feature_flows)
        )
        if body_delta is None:
            self._publish_visual_update_source(update_source)
            self._publish_range_feature_count()
            self._reset_tracks(gray, stamp)
            return
        self._publish_visual_update_source(update_source)
        self._publish_range_feature_count()

        body_delta = np.asarray(body_delta, dtype=float)
        c_yaw = math.cos(self.yaw)
        s_yaw = math.sin(self.yaw)
        world_delta = np.array(
            [
                c_yaw * body_delta[0] - s_yaw * body_delta[1],
                s_yaw * body_delta[0] + c_yaw * body_delta[1],
            ],
            dtype=float,
        )
        visual_velocity = world_delta / dt
        self._publish_flow_calibration_debug(
            flow,
            range_feature_flows,
            body_delta,
            world_delta,
            visual_velocity,
            dt,
        )
        if (
            not self.use_range_feature_3d
            and np.linalg.norm(visual_velocity) > self.max_visual_velocity_mps
        ):
            self._publish_visual_update_source('rejected_visual_velocity')
            self.prev_gray = gray
            self.prev_points = tracked_next.reshape(-1, 1, 2).astype(np.float32)
            self.prev_stamp = stamp
            self.prev_image_attitude_quat = self.attitude_quat.copy()
            return

        alpha = self.velocity_filter_alpha
        fused_velocity = (1.0 - alpha) * self.velocity[:2] + alpha * visual_velocity
        self.velocity[:2] = (
            (1.0 - self.visual_weight) * self.velocity[:2]
            + self.visual_weight * fused_velocity
        )
        self.position[:2] += world_delta
        self.visual_update_count += 1

        self._publish_estimate(msg.header.stamp)
        if self.publish_debug_image:
            self._publish_debug_image(
                gray,
                tracked_next,
                flow,
                msg.header.stamp,
                accepted_range_feature_flows,
                update_source,
            )

        self.prev_gray = gray
        self.prev_points = tracked_next.reshape(-1, 1, 2).astype(np.float32)
        self.prev_stamp = stamp
        self.prev_image_attitude_quat = self.attitude_quat.copy()
        self.frame_count += 1

    def _reset_tracks(self, gray, stamp):
        points = cv2.goodFeaturesToTrack(
            gray,
            maxCorners=self.max_features,
            qualityLevel=self.quality_level,
            minDistance=self.min_distance_px,
            blockSize=7,
        )
        self.prev_gray = gray
        self.prev_points = points
        self.prev_stamp = stamp
        self.prev_image_attitude_quat = self.attitude_quat.copy()

    def _center_corner_score(self, gray):
        block_size = max(3, self.center_corner_window)
        if block_size % 2 == 0:
            block_size += 1
        eig = cv2.cornerMinEigenVal(gray, blockSize=block_size, ksize=3)
        center_y = int(round(gray.shape[0] * 0.5))
        center_x = int(round(gray.shape[1] * 0.5))
        center_y = max(0, min(gray.shape[0] - 1, center_y))
        center_x = max(0, min(gray.shape[1] - 1, center_x))
        return float(eig[center_y, center_x])

    def _unit_vector(self, vector):
        vector = np.asarray(vector, dtype=float).reshape(3)
        norm = float(np.linalg.norm(vector))
        if norm <= 1.0e-9:
            return vector
        return vector / norm

    def _body_from_cv_camera_rotation(self, body_from_camera_link):
        # Gazebo camera sensors look along their link +X axis, while OpenCV
        # pixel rays use +Z as the optical axis, +X right, +Y down.
        link_from_cv = np.array([
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ], dtype=float)
        return body_from_camera_link @ link_from_cv

    def _project_body_point_to_image(self, point_body):
        if self.fx is None or self.fy is None or self.cx is None or self.cy is None:
            return None
        point_body = np.asarray(point_body, dtype=float).reshape(3)
        point_camera = self.camera_body_rotation_cv @ (
            point_body - self.camera_body_translation
        )
        if point_camera[2] <= 1.0e-6:
            return None
        u = self.fx * (point_camera[0] / point_camera[2]) + self.cx
        v = self.fy * (point_camera[1] / point_camera[2]) + self.cy
        if not (math.isfinite(u) and math.isfinite(v)):
            return None
        return np.array([u, v], dtype=float)

    def _pixel_ray_in_body(self, pixel):
        if self.fx is None or self.fy is None or self.cx is None or self.cy is None:
            return None
        u, v = np.asarray(pixel, dtype=float).reshape(2)
        ray_camera = np.array([
            (u - self.cx) / self.fx,
            (v - self.cy) / self.fy,
            1.0,
        ], dtype=float)
        return self._unit_vector(self.body_camera_rotation @ ray_camera)

    def _lidar_projection_pixel(self):
        if self.range_m is None:
            return None
        if not self.range_feature_use_attitude_projection:
            if self.cx is None or self.cy is None:
                return None
            return np.array([self.cx, self.cy], dtype=float)
        lidar_point_body = (
            self.lidar_body_translation + self.body_lidar_ray * float(self.range_m)
        )
        return self._project_body_point_to_image(lidar_point_body)

    def _image_depth_for_lidar_feature(self, pixel):
        ray_body = self._pixel_ray_in_body(pixel)
        if ray_body is None or self.range_m is None:
            return None
        terrain_point_body = (
            self.lidar_body_translation + self.body_lidar_ray * float(self.range_m)
        )
        relative_to_camera = terrain_point_body - self.camera_body_translation
        depth_along_ray = float(np.dot(relative_to_camera, ray_body))
        if not math.isfinite(depth_along_ray) or depth_along_ray <= 1.0e-6:
            return None
        return depth_along_ray

    def _world_from_body_rotation(self):
        if self.range_feature_use_attitude_projection:
            return self._estimated_world_from_body_rotation()
        return yaw_rotation_matrix(self.yaw)

    def _camera_pose_world(self, position=None, state=None):
        if position is None:
            position = self.ekf_state[0:3] if state is None else state[0:3]
        if state is not None and self.range_feature_use_attitude_projection:
            r_wb = self._estimated_world_from_body_rotation(state)
        else:
            r_wb = self._world_from_body_rotation()
        body_position_w = np.asarray(position, dtype=float).reshape(3)
        camera_position_w = body_position_w + r_wb @ self.camera_body_translation
        return camera_position_w, r_wb @ self.body_camera_rotation

    def _make_range_feature_anchor(self, pixel, depth):
        ray_body = self._pixel_ray_in_body(pixel)
        if ray_body is None:
            return None
        ray_camera = self.camera_body_rotation_cv @ ray_body
        ray_camera = self._unit_vector(ray_camera)
        camera_position_w, r_wc = self._camera_pose_world()
        anchor_ray_w = self._unit_vector(r_wc @ ray_camera)
        return {
            'anchor_camera_position_w': camera_position_w.copy(),
            'anchor_ray_w': anchor_ray_w,
            'anchor_alpha': float(ray_camera[0] / max(1.0e-9, ray_camera[2])),
            'anchor_beta': float(ray_camera[1] / max(1.0e-9, ray_camera[2])),
        }

    def _best_lidar_aligned_corner(self, gray):
        target = self._lidar_projection_pixel()
        if target is None:
            target = np.array([
                float(gray.shape[1] - 1) * 0.5,
                float(gray.shape[0] - 1) * 0.5,
            ], dtype=float)
        if not (
            -gray.shape[1] <= target[0] <= 2.0 * gray.shape[1]
            and -gray.shape[0] <= target[1] <= 2.0 * gray.shape[0]
        ):
            return None, 0.0, 0.0, None
        self._publish_float(self.debug_lidar_projection_u_pub, target[0])
        self._publish_float(self.debug_lidar_projection_v_pub, target[1])

        radius = max(
            1.0,
            self.range_feature_search_radius_px,
            self.range_feature_lidar_projection_radius_px,
        )
        max_offset = max(
            0.0,
            self.range_feature_max_center_offset_px,
            self.range_feature_lidar_projection_radius_px,
        )
        x0 = max(0, int(math.floor(target[0] - radius)))
        x1 = min(gray.shape[1], int(math.ceil(target[0] + radius + 1.0)))
        y0 = max(0, int(math.floor(target[1] - radius)))
        y1 = min(gray.shape[0], int(math.ceil(target[1] + radius + 1.0)))
        if x1 <= x0 or y1 <= y0:
            return None, 0.0, 0.0, target

        roi = gray[y0:y1, x0:x1]
        corners = cv2.goodFeaturesToTrack(
            roi,
            maxCorners=12,
            qualityLevel=max(1.0e-6, self.quality_level * 0.5),
            minDistance=max(2.0, self.range_feature_min_existing_separation_px * 0.5),
            blockSize=max(3, self.center_corner_window),
        )
        if corners is None:
            return None, 0.0, 0.0, target

        eig = cv2.cornerMinEigenVal(
            gray,
            blockSize=max(3, self.center_corner_window),
            ksize=3,
        )
        best_point = None
        best_score = 0.0
        best_offset = 0.0
        for corner in corners.reshape(-1, 2):
            point = np.array([float(corner[0]) + x0, float(corner[1]) + y0])
            offset = float(np.linalg.norm(point - target))
            if offset > max_offset:
                continue
            if self._is_near_active_range_feature(point):
                continue
            px = max(0, min(gray.shape[1] - 1, int(round(point[0]))))
            py = max(0, min(gray.shape[0] - 1, int(round(point[1]))))
            # Favor strong corners, but keep them close to the lidar beam.
            score = float(eig[py, px]) / (1.0 + offset / max(1.0, max_offset))
            if score > best_score:
                best_point = point
                best_score = score
                best_offset = offset
        if best_point is None:
            return None, 0.0, 0.0, target
        self._publish_float(self.debug_lidar_feature_offset_pub, best_offset)
        return best_point, best_score, best_offset, target

    def _is_near_active_range_feature(self, point):
        min_sep = max(0.0, self.range_feature_min_existing_separation_px)
        if min_sep <= 0.0:
            return False
        point = np.asarray(point, dtype=float).reshape(2)
        for feature in self.active_range_features:
            active_point = np.asarray(feature['point'], dtype=float).reshape(2)
            if np.linalg.norm(point - active_point) < min_sep:
                return True
        return False

    def _maybe_seed_range_feature(self, gray, stamp, center_score):
        self.center_score_history.append((stamp, center_score))
        lookahead = self.range_feature_peak_lookahead
        if lookahead > 0:
            if len(self.center_score_history) < lookahead + 1:
                return
            _, candidate_score = self.center_score_history[-lookahead - 1]
            future_scores = [
                score for _, score in list(self.center_score_history)[-lookahead:]
            ]
            if candidate_score < self.range_feature_min_score:
                return
            if not all(candidate_score > score for score in future_scores):
                return
        if self.last_range_feature_time is not None:
            if stamp - self.last_range_feature_time < self.range_feature_min_separation_sec:
                return
        if self.range_m is None:
            return
        if not self.range_feature_min_depth_m <= self.range_m <= self.range_feature_max_depth_m:
            return

        point, candidate_score, center_offset, projected_lidar_pixel = (
            self._best_lidar_aligned_corner(gray)
        )
        if point is None or candidate_score < self.range_feature_min_score:
            return
        depth_m = self._image_depth_for_lidar_feature(point)
        if depth_m is None:
            depth_m = float(self.range_m)
        if not self.range_feature_min_depth_m <= depth_m <= self.range_feature_max_depth_m:
            return
        point_msg = np.array([[point]], dtype=np.float32)
        offset_ratio = center_offset / max(
            1.0,
            self.range_feature_max_center_offset_px,
            self.range_feature_lidar_projection_radius_px,
        )
        depth_std = self.range_feature_depth_std * (
            1.0 + self.range_feature_offset_depth_uncertainty_scale * offset_ratio
        )
        feature = {
            'id': self.next_range_feature_id,
            'point': point_msg,
            'depth_m': float(depth_m),
            'depth_std': float(depth_std),
            'inverse_depth': 1.0 / float(depth_m),
            'state_index': None,
            'attitude_quat': self.attitude_quat.copy(),
            'age': 0,
            'created_time': stamp,
            'corner_score': candidate_score,
            'center_offset_px': center_offset,
            'lidar_range_m': float(self.range_m),
            'lidar_projection_px': (
                None
                if projected_lidar_pixel is None
                else np.asarray(projected_lidar_pixel, dtype=float).reshape(2)
            ),
        }
        anchor = self._make_range_feature_anchor(point, depth_m)
        if anchor is None:
            return
        feature.update(anchor)
        self._augment_range_feature_state(feature)
        self.next_range_feature_id += 1
        self.last_range_feature_time = stamp
        self.active_range_features.append(feature)
        if len(self.active_range_features) > self.max_range_features:
            old_feature = self.active_range_features.pop(0)
            self._remove_range_feature_state(old_feature)
        self._publish_range_feature_count()
        self._publish_float(self.range_feature_depth_pub, feature['depth_m'])
        self._publish_float(
            self.range_feature_center_offset_pub, feature['center_offset_px']
        )
        self.get_logger().info(
            'Seeded range-feature id=%d depth=%.3f m lidar=%.3f m inv_depth=%.6f score=%.6f offset=%.1f px'
            % (
                feature['id'],
                feature['depth_m'],
                feature['lidar_range_m'],
                feature['inverse_depth'],
                feature['corner_score'],
                feature['center_offset_px'],
            )
        )

    def _track_range_features(self, gray, current_attitude_quat, current_stamp):
        if self.prev_gray is None or not self.active_range_features:
            return []

        trackable_features = []
        deferred_features = []
        for feature in self.active_range_features:
            # A feature seeded from the current image cannot be tracked from
            # prev_gray yet. Keep it alive and start tracking on the next frame.
            if feature.get('created_time', 0.0) > self.prev_stamp + 1.0e-6:
                deferred_features.append(feature)
            else:
                trackable_features.append(feature)
        if not trackable_features:
            self.active_range_features = deferred_features
            return []

        prev_points = np.vstack([f['point'] for f in trackable_features]).astype(np.float32)
        next_points, status, _ = cv2.calcOpticalFlowPyrLK(
            self.prev_gray,
            gray,
            prev_points,
            None,
            winSize=(21, 21),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
        )
        if next_points is None or status is None:
            for feature in list(trackable_features):
                self._remove_range_feature_state(feature)
            self.active_range_features = deferred_features
            self._publish_range_feature_count()
            return []

        tracked = []
        survivors = list(deferred_features)
        removed = []
        for idx, feature in enumerate(trackable_features):
            if int(status[idx][0]) != 1:
                removed.append(feature)
                continue
            prev_pt = prev_points[idx].reshape(2)
            next_pt = next_points[idx].reshape(2)
            flow = next_pt - prev_pt
            if np.linalg.norm(flow) > self.max_flow_px:
                removed.append(feature)
                continue
            if not (0 <= next_pt[0] < gray.shape[1] and 0 <= next_pt[1] < gray.shape[0]):
                removed.append(feature)
                continue
            feature['point'] = next_points[idx].reshape(1, 1, 2).astype(np.float32)
            feature['age'] += 1
            if feature['age'] > self.range_feature_max_age_frames:
                removed.append(feature)
                continue
            survivors.append(feature)
            compensated_next = self._rotation_compensated_point(
                prev_pt,
                self.prev_image_attitude_quat,
                current_attitude_quat,
            )
            compensated_flow = next_pt - compensated_next
            tracked.append({
                'id': feature['id'],
                'prev': prev_pt,
                'curr': next_pt,
                'flow': flow,
                'compensated_flow': compensated_flow,
                'depth_m': feature['depth_m'],
                'inverse_depth': feature['inverse_depth'],
                'state_index': feature.get('state_index'),
                'age': feature['age'],
                'anchor_camera_position_w': feature.get('anchor_camera_position_w'),
                'anchor_ray_w': feature.get('anchor_ray_w'),
                'anchor_alpha': feature.get('anchor_alpha'),
                'anchor_beta': feature.get('anchor_beta'),
            })
        for feature in removed:
            self._remove_range_feature_state(feature)
        self.active_range_features = survivors
        return tracked

    def _filter_range_feature_flows(self, range_feature_flows, dt):
        if dt <= 1.0e-6:
            self._publish_int(self.debug_accepted_range_feature_count_pub, 0)
            self._publish_int(
                self.debug_rejected_range_feature_count_pub,
                len(range_feature_flows),
            )
            self._publish_float(self.debug_candidate_range_feature_pixel_rate_pub, 0.0)
            self._publish_float(self.debug_candidate_range_feature_speed_pub, 0.0)
            self._publish_range_feature_reject_reason('bad_dt')
            return []

        accepted = []
        rejected = 0
        pixel_rates = []
        body_speeds = []
        pixel_rate_rejections = 0
        body_speed_rejections = 0
        for track in range_feature_flows:
            du, dv = track.get('compensated_flow', track['flow'])
            pixel_rate = float(np.linalg.norm([du, dv]) / dt)
            depth = self._range_feature_depth(track)
            body_delta = self._camera_flow_to_body_delta(du, dv, depth)
            body_speed = float(np.linalg.norm(body_delta) / dt)
            pixel_rates.append(pixel_rate)
            body_speeds.append(body_speed)
            if pixel_rate > self.max_range_feature_pixel_rate_pxps:
                rejected += 1
                pixel_rate_rejections += 1
                continue
            if (
                not self.use_range_feature_3d
                and body_speed > self.max_range_feature_velocity_mps
            ):
                rejected += 1
                body_speed_rejections += 1
                continue
            accepted.append(track)

        if pixel_rates:
            self._publish_float(
                self.debug_candidate_range_feature_pixel_rate_pub,
                float(np.median(np.asarray(pixel_rates, dtype=float))),
            )
            self._publish_float(
                self.debug_candidate_range_feature_speed_pub,
                float(np.median(np.asarray(body_speeds, dtype=float))),
            )
        else:
            self._publish_float(self.debug_candidate_range_feature_pixel_rate_pub, 0.0)
            self._publish_float(self.debug_candidate_range_feature_speed_pub, 0.0)

        self._publish_int(self.debug_accepted_range_feature_count_pub, len(accepted))
        self._publish_int(self.debug_rejected_range_feature_count_pub, rejected)
        if not range_feature_flows:
            self._publish_range_feature_reject_reason('no_range_feature_tracks')
        elif accepted:
            self._publish_range_feature_reject_reason('accepted')
        elif pixel_rate_rejections >= body_speed_rejections:
            self._publish_range_feature_reject_reason('pixel_rate')
        else:
            self._publish_range_feature_reject_reason('body_speed')
        return accepted

    def _camera_flow_to_body_delta(self, du, dv, depth):
        camera_delta = np.array([
            self.flow_x_sign * (float(du) / self.fx) * float(depth),
            self.flow_y_sign * (float(dv) / self.fy) * float(depth),
            0.0,
        ], dtype=float)
        body_delta = self.body_camera_rotation @ camera_delta
        return body_delta[:2]

    def _metric_body_delta_from_flow(
        self,
        general_flow,
        range_feature_flows,
        raw_range_feature_count=0,
    ):
        if len(range_feature_flows) >= self.min_range_feature_tracks_for_update:
            deltas = []
            for track in range_feature_flows:
                du, dv = track.get('compensated_flow', track['flow'])
                depth = self._range_feature_depth(track)
                deltas.append(self._camera_flow_to_body_delta(du, dv, depth))
            return np.median(np.asarray(deltas, dtype=float), axis=0), 'attitude_range_feature'

        if raw_range_feature_count > 0:
            return None, 'rejected_range_feature_speed'

        if self.require_range_feature_update:
            return None, 'waiting_for_range_feature'

        du, dv = np.median(general_flow, axis=0)
        return np.array([0.0, 0.0]), 'no_range_feature_hold'

    def _rotation_compensated_point(self, point, prev_quat, curr_quat):
        if not self.compensate_camera_rotation or prev_quat is None or curr_quat is None:
            return np.asarray(point, dtype=float)
        r_prev = quaternion_to_rotation_matrix(prev_quat) @ self.camera_mount_rotation
        r_curr = quaternion_to_rotation_matrix(curr_quat) @ self.camera_mount_rotation
        # Rotation from previous camera orientation to current camera orientation.
        r_delta = r_curr.T @ r_prev
        pixel = np.array([float(point[0]), float(point[1]), 1.0], dtype=float)
        k = np.array([
            [self.fx, 0.0, self.cx],
            [0.0, self.fy, self.cy],
            [0.0, 0.0, 1.0],
        ], dtype=float)
        ray = np.linalg.inv(k) @ pixel
        rotated = r_delta @ ray
        if abs(rotated[2]) < 1e-9:
            return np.asarray(point, dtype=float)
        projected = k @ (rotated / rotated[2])
        return projected[:2]

    def _augment_range_feature_state(self, feature):
        depth = float(feature['depth_m'])
        if depth <= 1e-6:
            return
        inv_depth = 1.0 / depth
        depth_std = float(feature.get('depth_std', self.range_feature_depth_std))
        inv_depth_std = max(
            self.range_feature_inv_depth_std_floor,
            depth_std / (depth * depth),
        )
        old_n = len(self.ekf_state)
        self.ekf_state = np.append(self.ekf_state, inv_depth)
        cov = np.zeros((old_n + 1, old_n + 1), dtype=float)
        cov[:old_n, :old_n] = self.ekf_cov
        cov[old_n, old_n] = inv_depth_std * inv_depth_std
        self.ekf_cov = cov
        feature['state_index'] = old_n
        feature['inverse_depth'] = inv_depth

    def _remove_range_feature_state(self, feature):
        idx = feature.get('state_index')
        if idx is None or idx < self.base_state_size or idx >= len(self.ekf_state):
            feature['state_index'] = None
            return
        keep = [i for i in range(len(self.ekf_state)) if i != idx]
        self.ekf_state = self.ekf_state[keep]
        self.ekf_cov = self.ekf_cov[np.ix_(keep, keep)]
        feature['state_index'] = None
        for active in self.active_range_features:
            active_idx = active.get('state_index')
            if active_idx is not None and active_idx > idx:
                active['state_index'] = active_idx - 1

    def _range_feature_inverse_depth(self, feature_or_track):
        idx = feature_or_track.get('state_index')
        if idx is not None and 0 <= idx < len(self.ekf_state):
            rho = float(self.ekf_state[idx])
            if math.isfinite(rho) and rho > 1e-6:
                return rho
        rho = float(feature_or_track.get('inverse_depth', 0.0))
        if math.isfinite(rho) and rho > 1e-6:
            return rho
        depth = float(feature_or_track.get('depth_m', 0.0))
        if depth > 1e-6:
            return 1.0 / depth
        return None

    def _range_feature_depth(self, feature_or_track):
        rho = self._range_feature_inverse_depth(feature_or_track)
        if rho is None:
            return float(feature_or_track.get('depth_m', 0.0))
        return 1.0 / rho

    def _ekf_predict(self, dt, accel_body, gyro_body):
        n = len(self.ekf_state)
        if n < self.base_state_size:
            self.ekf_state = np.pad(self.ekf_state, (0, self.base_state_size - n))
            n = self.base_state_size
        accel_body = np.asarray(accel_body, dtype=float).reshape(3)
        gyro_body = np.asarray(gyro_body, dtype=float).reshape(3)
        r_wb = self._estimated_world_from_body_rotation(self.ekf_state)
        accel_world = r_wb @ (self.accel_weight * accel_body - self.ekf_state[9:12])
        if abs(accel_world[2]) > 5.0:
            accel_world[2] -= math.copysign(9.80665, accel_world[2])
        self.latest_world_accel = accel_world.copy()
        gyro_corrected = gyro_body - self.ekf_state[12:15]
        euler_rates = self._euler_rates_from_body_gyro(
            self.ekf_state[6:9], gyro_corrected
        )
        self.ekf_state[0:3] += self.ekf_state[3:6] * dt + 0.5 * accel_world * dt * dt
        self.ekf_state[3:6] += accel_world * dt
        self.ekf_state[6:9] += euler_rates * dt
        self.ekf_state[6] = wrap_angle(self.ekf_state[6])
        self.ekf_state[7] = max(
            -0.5 * math.pi + 1.0e-3,
            min(0.5 * math.pi - 1.0e-3, self.ekf_state[7]),
        )
        self.ekf_state[8] = wrap_angle(self.ekf_state[8])

        f = np.eye(n, dtype=float)
        for axis in range(3):
            pos_idx = axis
            vel_idx = 3 + axis
            bias_idx = 9 + axis
            f[pos_idx, vel_idx] = dt
            f[pos_idx, bias_idx] = -0.5 * dt * dt
            f[vel_idx, bias_idx] = -dt
            f[6 + axis, 12 + axis] = -dt
        accel_var = self.ekf_process_accel_std ** 2
        q_axis = accel_var * np.array([
            [0.25 * dt ** 4, 0.5 * dt ** 3],
            [0.5 * dt ** 3, dt ** 2],
        ])
        q = np.zeros((n, n), dtype=float)
        for axis in range(3):
            pos_idx = axis
            vel_idx = 3 + axis
            q[pos_idx, pos_idx] = q_axis[0, 0]
            q[pos_idx, vel_idx] = q_axis[0, 1]
            q[vel_idx, pos_idx] = q_axis[1, 0]
            q[vel_idx, vel_idx] = q_axis[1, 1]
        bias_var = self.ekf_accel_bias_rw_std ** 2
        for axis in range(3):
            q[9 + axis, 9 + axis] = bias_var * dt
        gyro_var = self.ekf_process_gyro_std ** 2
        gyro_bias_var = self.ekf_gyro_bias_rw_std ** 2
        for axis in range(3):
            q[6 + axis, 6 + axis] = gyro_var * dt
            q[12 + axis, 12 + axis] = gyro_bias_var * dt
        self.ekf_cov = f @ self.ekf_cov @ f.T + q

    def _predict_range_feature_normalized_pixel(self, track, state=None):
        if state is None:
            state = self.ekf_state
        idx = track.get('state_index')
        anchor_camera_position_w = track.get('anchor_camera_position_w')
        anchor_ray_w = track.get('anchor_ray_w')
        if (
            idx is None
            or idx >= len(state)
            or anchor_camera_position_w is None
            or anchor_ray_w is None
        ):
            return None
        rho = float(state[idx])
        if not math.isfinite(rho) or rho <= 1.0e-6:
            return None

        feature_w = (
            np.asarray(anchor_camera_position_w, dtype=float).reshape(3)
            + np.asarray(anchor_ray_w, dtype=float).reshape(3) / rho
        )
        camera_position_w, r_wc = self._camera_pose_world(state=state)
        point_c = r_wc.T @ (feature_w - camera_position_w)
        if point_c[2] <= 1.0e-6:
            return None
        return np.array([
            point_c[0] / point_c[2],
            point_c[1] / point_c[2],
        ], dtype=float)

    def _ekf_update_range_feature_reprojection(self, range_feature_flows):
        if not range_feature_flows:
            return False
        n = len(self.ekf_state)
        rows = []
        residuals = []
        variances = []
        residual_norms_px = []
        accepted_count = 0
        rejected_count = 0
        eps_xy = 1.0e-3
        eps_rho = 1.0e-5

        for track in range_feature_flows:
            idx = track.get('state_index')
            if idx is None or idx >= n:
                continue
            pred = self._predict_range_feature_normalized_pixel(track)
            if pred is None:
                continue
            curr = np.asarray(track['curr'], dtype=float).reshape(2)
            observed = np.array([
                (curr[0] - self.cx) / self.fx,
                (curr[1] - self.cy) / self.fy,
            ], dtype=float)
            residual = observed - pred
            residual_norm_px = math.hypot(
                residual[0] * self.fx,
                residual[1] * self.fy,
            )
            if residual_norm_px > self.range_feature_reprojection_gate_px:
                residual_norms_px.append(residual_norm_px)
                rejected_count += 1
                continue

            h = np.zeros((2, n), dtype=float)
            for state_idx, eps in (
                (0, eps_xy),
                (1, eps_xy),
                (2, eps_xy),
                (6, 1.0e-4),
                (7, 1.0e-4),
                (8, 1.0e-4),
                (idx, eps_rho),
            ):
                plus = self.ekf_state.copy()
                minus = self.ekf_state.copy()
                plus[state_idx] += eps
                minus[state_idx] -= eps
                if state_idx == idx:
                    plus[state_idx] = max(1.0e-6, plus[state_idx])
                    minus[state_idx] = max(1.0e-6, minus[state_idx])
                pred_plus = self._predict_range_feature_normalized_pixel(track, plus)
                pred_minus = self._predict_range_feature_normalized_pixel(track, minus)
                if pred_plus is None or pred_minus is None:
                    continue
                h[:, state_idx] = (pred_plus - pred_minus) / (
                    plus[state_idx] - minus[state_idx]
                )

            rows.append(h[0])
            residuals.append(residual[0])
            variances.append((self.range_feature_flow_rate_std_px / self.fx) ** 2)
            rows.append(h[1])
            residuals.append(residual[1])
            variances.append((self.range_feature_flow_rate_std_px / self.fy) ** 2)
            residual_norms_px.append(residual_norm_px)
            accepted_count += 1

        if residual_norms_px:
            self._publish_float(
                self.debug_reprojection_residual_pub,
                float(np.median(np.asarray(residual_norms_px, dtype=float))),
            )
        self._publish_int(self.debug_accepted_range_feature_count_pub, accepted_count)
        self._publish_int(self.debug_rejected_range_feature_count_pub, rejected_count)

        if not rows:
            self._publish_range_feature_reject_reason('reprojection_gate')
            return False

        r = np.diag(variances)
        h = np.vstack(rows)
        innovation = np.asarray(residuals, dtype=float)
        self._ekf_apply_update(h, innovation, r)

        for feature in self.active_range_features:
            idx = feature.get('state_index')
            if idx is not None and idx < len(self.ekf_state):
                rho = max(1.0e-6, float(self.ekf_state[idx]))
                feature['inverse_depth'] = rho
                feature['depth_m'] = 1.0 / rho
        return True

    def _ekf_update_range_z(self, range_z):
        if not math.isfinite(float(range_z)):
            return
        n = len(self.ekf_state)
        if n < self.base_state_size:
            return
        h = np.zeros((1, n), dtype=float)
        h[0, 2] = 1.0
        innovation = np.array([float(range_z) - float(self.ekf_state[2])], dtype=float)
        r = np.array([[max(1.0e-6, self.range_z_std) ** 2]], dtype=float)
        self._ekf_apply_update(h, innovation, r)

    def _ekf_update_attitude_measurement(self, roll, pitch, yaw):
        n = len(self.ekf_state)
        if n < self.base_state_size:
            return
        measured = np.array([float(roll), float(pitch), float(yaw)], dtype=float)
        predicted = self.ekf_state[6:9].copy()
        innovation = np.array(
            [
                wrap_angle(measured[0] - predicted[0]),
                wrap_angle(measured[1] - predicted[1]),
                wrap_angle(measured[2] - predicted[2]),
            ],
            dtype=float,
        )
        h = np.zeros((3, n), dtype=float)
        h[0, 6] = 1.0
        h[1, 7] = 1.0
        h[2, 8] = 1.0
        variance = max(1.0e-6, self.attitude_measurement_std) ** 2
        self._ekf_apply_update(h, innovation, np.eye(3, dtype=float) * variance)
        self.ekf_state[6] = wrap_angle(self.ekf_state[6])
        self.ekf_state[7] = max(
            -0.5 * math.pi + 1.0e-3,
            min(0.5 * math.pi - 1.0e-3, self.ekf_state[7]),
        )
        self.ekf_state[8] = wrap_angle(self.ekf_state[8])

    def _ekf_apply_update(self, h, innovation, r):
        s = h @ self.ekf_cov @ h.T + r
        try:
            k = self.ekf_cov @ h.T @ np.linalg.inv(s)
        except np.linalg.LinAlgError:
            self.get_logger().warn('Skipped EKF update due to singular covariance.')
            return
        self.ekf_state = self.ekf_state + k @ innovation
        identity = np.eye(len(self.ekf_state))
        # Joseph form keeps the small covariance matrix symmetric/positive in long runs.
        self.ekf_cov = (identity - k @ h) @ self.ekf_cov @ (identity - k @ h).T + k @ r @ k.T
        self.ekf_cov = 0.5 * (self.ekf_cov + self.ekf_cov.T)

    def _sync_state_from_ekf(self):
        self.position[:] = self.ekf_state[0:3]
        self.velocity[:] = self.ekf_state[3:6]
        self.roll, self.pitch, self.yaw = [
            float(v) for v in self.ekf_state[6:9]
        ]
        self.attitude_quat = euler_to_quaternion(self.roll, self.pitch, self.yaw)

    def _publish_flow_calibration_debug(
        self,
        general_flow,
        range_feature_flows,
        body_delta,
        world_delta,
        visual_velocity,
        dt,
    ):
        if dt <= 1.0e-6:
            return
        if len(general_flow) > 0:
            median_flow = np.median(np.asarray(general_flow, dtype=float), axis=0)
            self._publish_float(self.debug_flow_u_pub, median_flow[0])
            self._publish_float(self.debug_flow_v_pub, median_flow[1])

        if range_feature_flows:
            rf_flows = np.asarray(
                [
                    track.get('compensated_flow', track['flow'])
                    for track in range_feature_flows
                ],
                dtype=float,
            )
            median_rf_flow = np.median(rf_flows, axis=0)
            self._publish_float(self.debug_range_flow_u_pub, median_rf_flow[0])
            self._publish_float(self.debug_range_flow_v_pub, median_rf_flow[1])

        body_velocity = np.asarray(body_delta, dtype=float) / dt
        self._publish_float(self.debug_body_vx_pub, body_velocity[0])
        self._publish_float(self.debug_body_vy_pub, body_velocity[1])
        self._publish_float(self.debug_world_vx_pub, visual_velocity[0])
        self._publish_float(self.debug_world_vy_pub, visual_velocity[1])

        if self.ground_truth is None:
            return
        gt_vx = float(self.ground_truth.twist.twist.linear.x)
        gt_vy = float(self.ground_truth.twist.twist.linear.y)
        self._publish_float(self.debug_gt_vx_pub, gt_vx)
        self._publish_float(self.debug_gt_vy_pub, gt_vy)
        gt_velocity = np.array([gt_vx, gt_vy], dtype=float)
        visual_velocity = np.asarray(visual_velocity, dtype=float)
        denom = np.linalg.norm(gt_velocity) * np.linalg.norm(visual_velocity)
        if denom > 1.0e-6:
            self._publish_float(
                self.debug_velocity_sign_score_pub,
                float(np.dot(gt_velocity, visual_velocity) / denom),
            )

    def _publish_float(self, publisher, value):
        msg = Float32()
        msg.data = float(value)
        publisher.publish(msg)

    def _publish_int(self, publisher, value):
        msg = Int32()
        msg.data = int(value)
        publisher.publish(msg)

    def _publish_range_feature_count(self):
        msg = Int32()
        msg.data = int(len(self.active_range_features))
        self.range_feature_count_pub.publish(msg)

    def _publish_visual_update_source(self, source):
        msg = String()
        msg.data = str(source)
        self.visual_update_source_pub.publish(msg)

    def _publish_range_feature_reject_reason(self, reason):
        msg = String()
        msg.data = str(reason)
        self.debug_range_feature_reject_reason_pub.publish(msg)

    def _publish_estimate(self, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.world_frame
        odom.child_frame_id = self.body_frame
        odom.pose.pose.position.x = float(self.position[0])
        odom.pose.pose.position.y = float(self.position[1])
        odom.pose.pose.position.z = float(self.position[2])
        odom.pose.pose.orientation.x = float(self.attitude_quat[1])
        odom.pose.pose.orientation.y = float(self.attitude_quat[2])
        odom.pose.pose.orientation.z = float(self.attitude_quat[3])
        odom.pose.pose.orientation.w = float(self.attitude_quat[0])
        odom.twist.twist.linear.x = float(self.velocity[0])
        odom.twist.twist.linear.y = float(self.velocity[1])
        odom.twist.twist.linear.z = float(self.velocity[2])

        if self.use_range_feature_3d:
            odom.pose.covariance[0] = float(self.ekf_cov[0, 0])
            odom.pose.covariance[1] = float(self.ekf_cov[0, 1])
            odom.pose.covariance[2] = float(self.ekf_cov[0, 2])
            odom.pose.covariance[6] = float(self.ekf_cov[1, 0])
            odom.pose.covariance[7] = float(self.ekf_cov[1, 1])
            odom.pose.covariance[8] = float(self.ekf_cov[1, 2])
            odom.pose.covariance[12] = float(self.ekf_cov[2, 0])
            odom.pose.covariance[13] = float(self.ekf_cov[2, 1])
            odom.pose.covariance[14] = float(self.ekf_cov[2, 2])
            odom.pose.covariance[21] = float(self.ekf_cov[6, 6])
            odom.pose.covariance[28] = float(self.ekf_cov[7, 7])
            odom.pose.covariance[35] = float(self.ekf_cov[8, 8])
            odom.twist.covariance[0] = float(self.ekf_cov[3, 3])
            odom.twist.covariance[1] = float(self.ekf_cov[3, 4])
            odom.twist.covariance[2] = float(self.ekf_cov[3, 5])
            odom.twist.covariance[6] = float(self.ekf_cov[4, 3])
            odom.twist.covariance[7] = float(self.ekf_cov[4, 4])
            odom.twist.covariance[8] = float(self.ekf_cov[4, 5])
            odom.twist.covariance[12] = float(self.ekf_cov[5, 3])
            odom.twist.covariance[13] = float(self.ekf_cov[5, 4])
            odom.twist.covariance[14] = float(self.ekf_cov[5, 5])
        else:
            odom.pose.covariance[0] = 0.30
            odom.pose.covariance[7] = 0.30
            odom.twist.covariance[0] = 0.25
            odom.twist.covariance[7] = 0.25
            odom.pose.covariance[14] = 0.05
            odom.twist.covariance[14] = 0.20
        if not self.use_range_feature_3d:
            odom.pose.covariance[35] = 0.20
        self.odom_pub.publish(odom)

        stamped_pose = PoseStamped()
        stamped_pose.header = odom.header
        stamped_pose.pose = odom.pose.pose
        self.path.append(stamped_pose)

        path_msg = Path()
        path_msg.header = odom.header
        path_msg.poses = list(self.path)
        self.path_pub.publish(path_msg)

        if self.ground_truth is not None:
            error = Vector3Stamped()
            error.header = odom.header
            error.vector.x = odom.pose.pose.position.x - self.ground_truth.pose.pose.position.x
            error.vector.y = odom.pose.pose.position.y - self.ground_truth.pose.pose.position.y
            error.vector.z = self._ground_truth_z_error(odom)
            self.error_pub.publish(error)

    def _ground_truth_z_error(self, odom):
        if self.ground_truth_error_z_mode == 'raw':
            return odom.pose.pose.position.z - self.ground_truth.pose.pose.position.z
        if self.ground_truth_error_z_mode in ('zero', 'ignore_zero'):
            return 0.0
        return float('nan')

    def _publish_debug_image(self, gray, points, flow, stamp, range_feature_flows, update_source):
        color = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        for point, vec in zip(points, flow):
            x, y = point
            px = int(round(x))
            py = int(round(y))
            cv2.circle(color, (px, py), 2, (0, 255, 0), -1)
            cv2.line(
                color,
                (px, py),
                (int(round(px - vec[0])), int(round(py - vec[1]))),
                (0, 180, 255),
                1,
            )
        for track in range_feature_flows:
            x, y = track['curr']
            px = int(round(x))
            py = int(round(y))
            cv2.circle(color, (px, py), 6, (0, 0, 255), 2)
            cv2.putText(
                color,
                f"rf{track['id']} {track['depth_m']:.1f}m",
                (px + 6, py - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 0, 255),
                1,
                cv2.LINE_AA,
            )
        lidar_pixel = self._lidar_projection_pixel()
        if lidar_pixel is None:
            lidar_pixel = np.array([
                float(gray.shape[1]) * 0.5,
                float(gray.shape[0]) * 0.5,
            ], dtype=float)
        center = (int(round(lidar_pixel[0])), int(round(lidar_pixel[1])))
        cv2.circle(
            color,
            center,
            int(round(self.range_feature_lidar_projection_radius_px)),
            (0, 80, 255),
            1,
        )
        cv2.drawMarker(
            color,
            center,
            (0, 0, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=18,
            thickness=2,
        )
        cv2.putText(
            color,
            f'agl={self.position[2]:.2f}m rf={len(self.active_range_features)} {update_source}',
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        for feature in self.active_range_features:
            point = np.asarray(feature['point'], dtype=float).reshape(2)
            px = int(round(point[0]))
            py = int(round(point[1]))
            cv2.circle(color, (px, py), 4, (255, 0, 255), 1)
        out = self.bridge.cv2_to_imgmsg(color, encoding='bgr8')
        out.header.stamp = stamp
        out.header.frame_id = self.body_frame
        self.debug_image_pub.publish(out)

    def ground_truth_cb(self, msg):
        self.ground_truth = msg


def main(args=None):
    rclpy.init(args=args)
    node = RvioPocNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except RuntimeError:
            pass


if __name__ == '__main__':
    main()
