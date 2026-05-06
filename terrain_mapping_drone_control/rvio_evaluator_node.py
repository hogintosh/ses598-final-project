#!/usr/bin/env python3

import csv
import math
import os
from datetime import datetime

import rclpy
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String


def stamp_to_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def finite_range_from_scan(msg):
    valid_ranges = [
        float(r)
        for r in msg.ranges
        if math.isfinite(r) and msg.range_min <= r <= msg.range_max
    ]
    if not valid_ranges:
        return None
    return min(valid_ranges)


class RvioEvaluatorNode(Node):
    """Evaluate RVIO in a relative local frame against Gazebo and lidar."""

    def __init__(self):
        super().__init__('rvio_evaluator_node')

        self.declare_parameter('rvio_odom_topic', '/rvio/odometry')
        self.declare_parameter('ground_truth_topic', '/sim/ground_truth/vehicle_odometry')
        self.declare_parameter('range_topic', '/drone/down_rangefinder')
        self.declare_parameter('mission_state_topic', '/mission/state')
        self.declare_parameter('start_on_mission_state', '')
        self.declare_parameter('relative_error_topic', '/rvio_eval/relative_error')
        self.declare_parameter('velocity_error_topic', '/rvio_eval/velocity_error')
        self.declare_parameter('xy_error_norm_topic', '/rvio_eval/xy_error_norm')
        self.declare_parameter('velocity_error_norm_topic', '/rvio_eval/velocity_error_norm')
        self.declare_parameter('agl_error_topic', '/rvio_eval/agl_error')
        self.declare_parameter('xy_rmse_topic', '/rvio_eval/xy_rmse')
        self.declare_parameter('velocity_rmse_topic', '/rvio_eval/velocity_rmse')
        self.declare_parameter('agl_rmse_topic', '/rvio_eval/agl_rmse')
        self.declare_parameter('drift_percent_topic', '/rvio_eval/drift_percent')
        self.declare_parameter('gt_path_length_topic', '/rvio_eval/path_length_gt')
        self.declare_parameter('rvio_path_length_topic', '/rvio_eval/path_length_rvio')
        self.declare_parameter('metrics_frame', 'map')
        self.declare_parameter('write_csv', False)
        self.declare_parameter('csv_directory', '~/.ros/rvio_eval')
        self.declare_parameter('log_period_sec', 2.0)
        self.declare_parameter('plot_publish_rate_hz', 10.0)

        self.rvio_odom_topic = str(self.get_parameter('rvio_odom_topic').value)
        self.ground_truth_topic = str(self.get_parameter('ground_truth_topic').value)
        self.range_topic = str(self.get_parameter('range_topic').value)
        self.mission_state_topic = str(
            self.get_parameter('mission_state_topic').value
        )
        self.start_on_mission_state = str(
            self.get_parameter('start_on_mission_state').value
        ).upper()
        self.relative_error_topic = str(
            self.get_parameter('relative_error_topic').value
        )
        self.velocity_error_topic = str(
            self.get_parameter('velocity_error_topic').value
        )
        self.xy_error_norm_topic = str(
            self.get_parameter('xy_error_norm_topic').value
        )
        self.velocity_error_norm_topic = str(
            self.get_parameter('velocity_error_norm_topic').value
        )
        self.agl_error_topic = str(self.get_parameter('agl_error_topic').value)
        self.xy_rmse_topic = str(self.get_parameter('xy_rmse_topic').value)
        self.velocity_rmse_topic = str(
            self.get_parameter('velocity_rmse_topic').value
        )
        self.agl_rmse_topic = str(self.get_parameter('agl_rmse_topic').value)
        self.drift_percent_topic = str(
            self.get_parameter('drift_percent_topic').value
        )
        self.gt_path_length_topic = str(
            self.get_parameter('gt_path_length_topic').value
        )
        self.rvio_path_length_topic = str(
            self.get_parameter('rvio_path_length_topic').value
        )
        self.metrics_frame = str(self.get_parameter('metrics_frame').value)
        self.write_csv = bool(self.get_parameter('write_csv').value)
        self.csv_directory = os.path.expanduser(
            str(self.get_parameter('csv_directory').value)
        )
        self.log_period_sec = float(self.get_parameter('log_period_sec').value)
        self.plot_publish_rate_hz = float(
            self.get_parameter('plot_publish_rate_hz').value
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
        plot_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.relative_error_pub = self.create_publisher(
            Vector3Stamped, self.relative_error_topic, 10
        )
        self.velocity_error_pub = self.create_publisher(
            Vector3Stamped, self.velocity_error_topic, 10
        )
        self.xy_error_norm_pub = self.create_publisher(
            Float32, self.xy_error_norm_topic, 10
        )
        self.velocity_error_norm_pub = self.create_publisher(
            Float32, self.velocity_error_norm_topic, 10
        )
        self.agl_error_pub = self.create_publisher(
            Float32, self.agl_error_topic, 10
        )
        self.xy_rmse_pub = self.create_publisher(Float32, self.xy_rmse_topic, 10)
        self.velocity_rmse_pub = self.create_publisher(
            Float32, self.velocity_rmse_topic, 10
        )
        self.agl_rmse_pub = self.create_publisher(Float32, self.agl_rmse_topic, 10)
        self.drift_percent_pub = self.create_publisher(
            Float32, self.drift_percent_topic, 10
        )
        self.gt_path_length_pub = self.create_publisher(
            Float32, self.gt_path_length_topic, 10
        )
        self.rvio_path_length_pub = self.create_publisher(
            Float32, self.rvio_path_length_topic, 10
        )
        self.plot_estimated_x_pub = self.create_publisher(
            Float32, '/rvio_plot/estimated_x', plot_qos
        )
        self.plot_ground_truth_x_pub = self.create_publisher(
            Float32, '/rvio_plot/ground_truth_x', plot_qos
        )
        self.plot_estimated_y_pub = self.create_publisher(
            Float32, '/rvio_plot/estimated_y', plot_qos
        )
        self.plot_ground_truth_y_pub = self.create_publisher(
            Float32, '/rvio_plot/ground_truth_y', plot_qos
        )
        self.plot_estimated_vx_pub = self.create_publisher(
            Float32, '/rvio_plot/estimated_vx', plot_qos
        )
        self.plot_ground_truth_vx_pub = self.create_publisher(
            Float32, '/rvio_plot/ground_truth_vx', plot_qos
        )
        self.plot_estimated_vy_pub = self.create_publisher(
            Float32, '/rvio_plot/estimated_vy', plot_qos
        )
        self.plot_ground_truth_vy_pub = self.create_publisher(
            Float32, '/rvio_plot/ground_truth_vy', plot_qos
        )
        self.plot_final_estimated_x_pub = self.create_publisher(
            Float32, '/rvio_plot/final_estimated_x', plot_qos
        )
        self.plot_final_ground_truth_x_pub = self.create_publisher(
            Float32, '/rvio_plot/final_ground_truth_x', plot_qos
        )
        self.plot_final_estimated_y_pub = self.create_publisher(
            Float32, '/rvio_plot/final_estimated_y', plot_qos
        )
        self.plot_final_ground_truth_y_pub = self.create_publisher(
            Float32, '/rvio_plot/final_ground_truth_y', plot_qos
        )

        self.create_subscription(
            Odometry, self.rvio_odom_topic, self.rvio_odom_cb, 10
        )
        self.create_subscription(
            Odometry, self.ground_truth_topic, self.ground_truth_cb, 10
        )
        self.create_subscription(LaserScan, self.range_topic, self.range_cb, sensor_qos)
        self.create_subscription(
            String, self.mission_state_topic, self.mission_state_cb, state_qos
        )

        self.latest_gt = None
        self.latest_range = None
        self.latest_rvio_rel = None
        self.latest_rvio_velocity_xy = None
        self.latest_gt_rel = None
        self.latest_gt_velocity_xy = None
        self.latest_plot_sample = None
        self.evaluator_active = self.start_on_mission_state == ''
        self.last_mission_state = ''
        self._reset_metrics()
        if self.plot_publish_rate_hz > 0.0:
            self.create_timer(
                1.0 / self.plot_publish_rate_hz,
                self.publish_latest_plot_sample,
                clock=Clock(clock_type=ClockType.STEADY_TIME),
            )

        self.csv_file = None
        self.csv_writer = None
        self.csv_path = None
        if self.write_csv:
            self._open_csv()

        if not self.evaluator_active:
            self.get_logger().info(
                'RVIO evaluator waiting for mission state %s on %s'
                % (self.start_on_mission_state, self.mission_state_topic)
            )
        self.get_logger().info(
            'RVIO evaluator ready: rvio=%s ground_truth=%s range=%s'
            % (self.rvio_odom_topic, self.ground_truth_topic, self.range_topic)
        )

    def _reset_metrics(self):
        self.rvio_origin = None
        self.gt_origin = None
        self.prev_rvio_rel = None
        self.prev_gt_rel = None
        self.sample_count = 0
        self.xy_squared_error_sum = 0.0
        self.velocity_squared_error_sum = 0.0
        self.agl_squared_error_sum = 0.0
        self.gt_path_length = 0.0
        self.rvio_path_length = 0.0
        self.last_log_time = None
        self.latest_rvio_rel = None
        self.latest_rvio_velocity_xy = None
        self.latest_gt_rel = None
        self.latest_gt_velocity_xy = None
        self.latest_plot_sample = None

    def _open_csv(self):
        os.makedirs(self.csv_directory, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(self.csv_directory, f'rvio_eval_{stamp}.csv')
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'stamp_sec',
            'rvio_rel_x',
            'rvio_rel_y',
            'rvio_agl',
            'gt_rel_x',
            'gt_rel_y',
            'gt_world_z',
            'lidar_agl',
            'rvio_vx',
            'rvio_vy',
            'gt_vx',
            'gt_vy',
            'xy_error_x',
            'xy_error_y',
            'xy_error_norm',
            'velocity_error_x',
            'velocity_error_y',
            'velocity_error_norm',
            'agl_error',
            'xy_rmse',
            'velocity_rmse',
            'agl_rmse',
            'drift_percent',
            'gt_path_length',
            'rvio_path_length',
        ])
        self.get_logger().info(f'Writing RVIO evaluation CSV: {self.csv_path}')

    def range_cb(self, msg):
        measured = finite_range_from_scan(msg)
        if measured is not None:
            self.latest_range = measured

    def ground_truth_cb(self, msg):
        self.latest_gt = msg
        if self.evaluator_active and self.gt_origin is None:
            self._lock_ground_truth_origin()
        if not self.evaluator_active or self.gt_origin is None:
            return

        gt_rel = (
            float(msg.pose.pose.position.x) - self.gt_origin[0],
            float(msg.pose.pose.position.y) - self.gt_origin[1],
        )
        gt_velocity_xy = (
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.linear.y),
        )
        self.latest_gt_rel = gt_rel
        self.latest_gt_velocity_xy = gt_velocity_xy
        self._publish_ground_truth_plot_topics(gt_rel, gt_velocity_xy)

        if self.latest_rvio_rel is not None and self.latest_rvio_velocity_xy is not None:
            self.latest_plot_sample = (
                self.latest_rvio_rel,
                gt_rel,
                self.latest_rvio_velocity_xy,
                gt_velocity_xy,
            )

    def mission_state_cb(self, msg):
        state = str(msg.data).upper()
        self.last_mission_state = state
        if self.evaluator_active or state != self.start_on_mission_state:
            return

        self._reset_metrics()
        self.evaluator_active = True
        self.rvio_origin = (0.0, 0.0)
        self.prev_rvio_rel = (0.0, 0.0)
        self.get_logger().info(
            'RVIO evaluator activated at mission state %s; ground-truth origin will lock to the SURVEY-start Gazebo pose.'
            % state
        )
        self._lock_ground_truth_origin()

    def _lock_ground_truth_origin(self):
        if self.latest_gt is None or self.gt_origin is not None:
            return

        self.gt_origin = (
            float(self.latest_gt.pose.pose.position.x),
            float(self.latest_gt.pose.pose.position.y),
        )
        self.prev_gt_rel = (0.0, 0.0)
        gt_velocity_xy = (
            float(self.latest_gt.twist.twist.linear.x),
            float(self.latest_gt.twist.twist.linear.y),
        )
        self._publish_plot_topics(
            (0.0, 0.0),
            (0.0, 0.0),
            (0.0, 0.0),
            gt_velocity_xy,
        )
        self.get_logger().info(
            'RVIO evaluator ground-truth origin locked at SURVEY start: '
            f'gt=({self.gt_origin[0]:.3f}, {self.gt_origin[1]:.3f}); '
            'plot ground truth starts at (0.000, 0.000).'
        )

    def rvio_odom_cb(self, msg):
        if not self.evaluator_active:
            return
        if self.latest_gt is None or self.latest_range is None:
            return
        if self.gt_origin is None:
            self._lock_ground_truth_origin()
        if self.gt_origin is None:
            return

        rvio_xy = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )
        gt_xy = (
            float(self.latest_gt.pose.pose.position.x),
            float(self.latest_gt.pose.pose.position.y),
        )

        if self.rvio_origin is None:
            self.rvio_origin = (0.0, 0.0)
            self.prev_rvio_rel = (0.0, 0.0)
            self.get_logger().info(
                'RVIO evaluator using RVIO estimator frame origin at (0.000, 0.000); '
                f'first rvio sample=({rvio_xy[0]:.3f}, {rvio_xy[1]:.3f}).'
            )

        rvio_rel = (
            rvio_xy[0] - self.rvio_origin[0],
            rvio_xy[1] - self.rvio_origin[1],
        )
        gt_rel = (
            gt_xy[0] - self.gt_origin[0],
            gt_xy[1] - self.gt_origin[1],
        )

        xy_error_x = rvio_rel[0] - gt_rel[0]
        xy_error_y = rvio_rel[1] - gt_rel[1]
        xy_error_norm = math.hypot(xy_error_x, xy_error_y)
        rvio_agl = float(msg.pose.pose.position.z)
        agl_error = rvio_agl - self.latest_range
        rvio_velocity_xy = (
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.linear.y),
        )
        gt_velocity_xy = (
            float(self.latest_gt.twist.twist.linear.x),
            float(self.latest_gt.twist.twist.linear.y),
        )
        self.latest_rvio_rel = rvio_rel
        self.latest_rvio_velocity_xy = rvio_velocity_xy
        self.latest_gt_rel = gt_rel
        self.latest_gt_velocity_xy = gt_velocity_xy
        velocity_error_x = rvio_velocity_xy[0] - gt_velocity_xy[0]
        velocity_error_y = rvio_velocity_xy[1] - gt_velocity_xy[1]
        velocity_error_norm = math.hypot(velocity_error_x, velocity_error_y)
        self._publish_plot_topics(
            rvio_rel, gt_rel, rvio_velocity_xy, gt_velocity_xy
        )

        self.gt_path_length += self._segment_length(self.prev_gt_rel, gt_rel)
        self.rvio_path_length += self._segment_length(self.prev_rvio_rel, rvio_rel)
        self.prev_gt_rel = gt_rel
        self.prev_rvio_rel = rvio_rel
        if self.last_log_time is None:
            self.last_log_time = stamp_to_seconds(msg.header.stamp)

        self.sample_count += 1
        self.xy_squared_error_sum += xy_error_norm * xy_error_norm
        self.velocity_squared_error_sum += velocity_error_norm * velocity_error_norm
        self.agl_squared_error_sum += agl_error * agl_error
        xy_rmse = math.sqrt(self.xy_squared_error_sum / self.sample_count)
        velocity_rmse = math.sqrt(
            self.velocity_squared_error_sum / self.sample_count
        )
        agl_rmse = math.sqrt(self.agl_squared_error_sum / self.sample_count)
        drift_percent = 0.0
        if self.gt_path_length > 1e-6:
            drift_percent = 100.0 * xy_error_norm / self.gt_path_length

        stamp = msg.header.stamp
        self._publish_metrics(
            stamp,
            xy_error_x,
            xy_error_y,
            agl_error,
            xy_error_norm,
            velocity_error_x,
            velocity_error_y,
            velocity_error_norm,
            xy_rmse,
            velocity_rmse,
            agl_rmse,
            drift_percent,
        )
        self._write_csv_row(
            stamp,
            rvio_rel,
            rvio_agl,
            gt_rel,
            float(self.latest_gt.pose.pose.position.z),
            self.latest_range,
            rvio_velocity_xy,
            gt_velocity_xy,
            xy_error_x,
            xy_error_y,
            xy_error_norm,
            velocity_error_x,
            velocity_error_y,
            velocity_error_norm,
            agl_error,
            xy_rmse,
            velocity_rmse,
            agl_rmse,
            drift_percent,
        )
        self._maybe_log_summary(
            stamp,
            xy_error_norm,
            velocity_error_norm,
            agl_error,
            xy_rmse,
            velocity_rmse,
            agl_rmse,
            drift_percent,
        )

    def _segment_length(self, prev_xy, curr_xy):
        if prev_xy is None:
            return 0.0
        return math.hypot(curr_xy[0] - prev_xy[0], curr_xy[1] - prev_xy[1])

    def _publish_metrics(
        self,
        stamp,
        xy_error_x,
        xy_error_y,
        agl_error,
        xy_error_norm,
        velocity_error_x,
        velocity_error_y,
        velocity_error_norm,
        xy_rmse,
        velocity_rmse,
        agl_rmse,
        drift_percent,
    ):
        error_msg = Vector3Stamped()
        error_msg.header.stamp = stamp
        error_msg.header.frame_id = self.metrics_frame
        error_msg.vector.x = float(xy_error_x)
        error_msg.vector.y = float(xy_error_y)
        error_msg.vector.z = float(agl_error)
        self.relative_error_pub.publish(error_msg)

        velocity_msg = Vector3Stamped()
        velocity_msg.header.stamp = stamp
        velocity_msg.header.frame_id = self.metrics_frame
        velocity_msg.vector.x = float(velocity_error_x)
        velocity_msg.vector.y = float(velocity_error_y)
        velocity_msg.vector.z = 0.0
        self.velocity_error_pub.publish(velocity_msg)

        self._publish_float(self.xy_error_norm_pub, xy_error_norm)
        self._publish_float(self.velocity_error_norm_pub, velocity_error_norm)
        self._publish_float(self.agl_error_pub, agl_error)
        self._publish_float(self.xy_rmse_pub, xy_rmse)
        self._publish_float(self.velocity_rmse_pub, velocity_rmse)
        self._publish_float(self.agl_rmse_pub, agl_rmse)
        self._publish_float(self.drift_percent_pub, drift_percent)
        self._publish_float(self.gt_path_length_pub, self.gt_path_length)
        self._publish_float(self.rvio_path_length_pub, self.rvio_path_length)

    def _publish_plot_topics(
        self,
        rvio_rel,
        gt_rel,
        rvio_velocity_xy,
        gt_velocity_xy,
    ):
        self.latest_plot_sample = (
            tuple(rvio_rel),
            tuple(gt_rel),
            tuple(rvio_velocity_xy),
            tuple(gt_velocity_xy),
        )
        self._publish_plot_sample(self.latest_plot_sample)

    def _publish_ground_truth_plot_topics(self, gt_rel, gt_velocity_xy):
        self._publish_float(self.plot_ground_truth_x_pub, gt_rel[0])
        self._publish_float(self.plot_ground_truth_y_pub, gt_rel[1])
        self._publish_float(self.plot_ground_truth_vx_pub, gt_velocity_xy[0])
        self._publish_float(self.plot_ground_truth_vy_pub, gt_velocity_xy[1])
        self._publish_float(self.plot_final_ground_truth_x_pub, gt_rel[0])
        self._publish_float(self.plot_final_ground_truth_y_pub, gt_rel[1])

    def publish_latest_plot_sample(self):
        if self.latest_plot_sample is None:
            if self.latest_gt_rel is not None and self.latest_gt_velocity_xy is not None:
                self._publish_ground_truth_plot_topics(
                    self.latest_gt_rel,
                    self.latest_gt_velocity_xy,
                )
            return
        self._publish_plot_sample(self.latest_plot_sample)

    def _publish_plot_sample(self, sample):
        rvio_rel, gt_rel, rvio_velocity_xy, gt_velocity_xy = sample
        self._publish_float(self.plot_estimated_x_pub, rvio_rel[0])
        self._publish_float(self.plot_ground_truth_x_pub, gt_rel[0])
        self._publish_float(self.plot_estimated_y_pub, rvio_rel[1])
        self._publish_float(self.plot_ground_truth_y_pub, gt_rel[1])
        self._publish_float(self.plot_estimated_vx_pub, rvio_velocity_xy[0])
        self._publish_float(self.plot_ground_truth_vx_pub, gt_velocity_xy[0])
        self._publish_float(self.plot_estimated_vy_pub, rvio_velocity_xy[1])
        self._publish_float(self.plot_ground_truth_vy_pub, gt_velocity_xy[1])
        # These "final" topics are continuously overwritten with the latest
        # sample. At the end of the mission they hold the terminal comparison.
        self._publish_float(self.plot_final_estimated_x_pub, rvio_rel[0])
        self._publish_float(self.plot_final_ground_truth_x_pub, gt_rel[0])
        self._publish_float(self.plot_final_estimated_y_pub, rvio_rel[1])
        self._publish_float(self.plot_final_ground_truth_y_pub, gt_rel[1])

    def _publish_float(self, publisher, value):
        msg = Float32()
        msg.data = float(value)
        publisher.publish(msg)

    def _write_csv_row(
        self,
        stamp,
        rvio_rel,
        rvio_agl,
        gt_rel,
        gt_world_z,
        lidar_agl,
        rvio_velocity_xy,
        gt_velocity_xy,
        xy_error_x,
        xy_error_y,
        xy_error_norm,
        velocity_error_x,
        velocity_error_y,
        velocity_error_norm,
        agl_error,
        xy_rmse,
        velocity_rmse,
        agl_rmse,
        drift_percent,
    ):
        if self.csv_writer is None:
            return
        self.csv_writer.writerow([
            f'{stamp_to_seconds(stamp):.9f}',
            f'{rvio_rel[0]:.9f}',
            f'{rvio_rel[1]:.9f}',
            f'{rvio_agl:.9f}',
            f'{gt_rel[0]:.9f}',
            f'{gt_rel[1]:.9f}',
            f'{gt_world_z:.9f}',
            f'{lidar_agl:.9f}',
            f'{rvio_velocity_xy[0]:.9f}',
            f'{rvio_velocity_xy[1]:.9f}',
            f'{gt_velocity_xy[0]:.9f}',
            f'{gt_velocity_xy[1]:.9f}',
            f'{xy_error_x:.9f}',
            f'{xy_error_y:.9f}',
            f'{xy_error_norm:.9f}',
            f'{velocity_error_x:.9f}',
            f'{velocity_error_y:.9f}',
            f'{velocity_error_norm:.9f}',
            f'{agl_error:.9f}',
            f'{xy_rmse:.9f}',
            f'{velocity_rmse:.9f}',
            f'{agl_rmse:.9f}',
            f'{drift_percent:.9f}',
            f'{self.gt_path_length:.9f}',
            f'{self.rvio_path_length:.9f}',
        ])
        self.csv_file.flush()

    def _maybe_log_summary(
        self,
        stamp,
        xy_error_norm,
        velocity_error_norm,
        agl_error,
        xy_rmse,
        velocity_rmse,
        agl_rmse,
        drift_percent,
    ):
        now = stamp_to_seconds(stamp)
        if self.last_log_time is not None and now - self.last_log_time < self.log_period_sec:
            return
        self.last_log_time = now
        self.get_logger().info(
            'RVIO metrics: '
            f'xy_err={xy_error_norm:.3f} m, vel_err={velocity_error_norm:.3f} m/s, '
            f'agl_err={agl_error:.3f} m, xy_rmse={xy_rmse:.3f} m, '
            f'vel_rmse={velocity_rmse:.3f} m/s, agl_rmse={agl_rmse:.3f} m, '
            f'drift={drift_percent:.2f}%'
        )

    def destroy_node(self):
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RvioEvaluatorNode()
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
