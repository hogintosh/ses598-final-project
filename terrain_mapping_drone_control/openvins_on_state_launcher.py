#!/usr/bin/env python3

import subprocess

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class OpenVinsOnStateLauncher(Node):
    """Start OpenVINS only after the mission reaches the estimator start state."""

    def __init__(self):
        super().__init__('openvins_on_state_launcher')

        self.declare_parameter('mission_state_topic', '/mission/state')
        self.declare_parameter('start_on_state', 'SURVEY')
        self.declare_parameter('config_path', '')
        self.declare_parameter('topic_imu', '/openvins/imu')
        self.declare_parameter('topic_camera0', '/drone/down_mono')
        self.declare_parameter('topic_range', '/drone/down_rangefinder')
        self.declare_parameter('verbosity', 'INFO')
        self.declare_parameter('range_feature_enable', True)
        self.declare_parameter('node_name', 'openvins_range_rvio')

        self.mission_state_topic = str(
            self.get_parameter('mission_state_topic').value
        )
        self.start_on_state = str(self.get_parameter('start_on_state').value).upper()
        self.config_path = str(self.get_parameter('config_path').value)
        self.topic_imu = str(self.get_parameter('topic_imu').value)
        self.topic_camera0 = str(self.get_parameter('topic_camera0').value)
        self.topic_range = str(self.get_parameter('topic_range').value)
        self.verbosity = str(self.get_parameter('verbosity').value)
        self.range_feature_enable = bool(
            self.get_parameter('range_feature_enable').value
        )
        self.node_name = str(self.get_parameter('node_name').value)

        self.process = None

        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            String, self.mission_state_topic, self.mission_state_cb, state_qos
        )

        if not self.start_on_state:
            self.get_logger().info('No start state configured; launching OpenVINS now.')
            self.launch_openvins()
        else:
            self.get_logger().info(
                'OpenVINS launcher waiting for %s on %s.'
                % (self.start_on_state, self.mission_state_topic)
            )

    def mission_state_cb(self, msg):
        if self.process is not None:
            return
        state = str(msg.data).upper()
        if state != self.start_on_state:
            return
        self.get_logger().info('Mission entered %s; launching OpenVINS.' % state)
        self.launch_openvins()

    def launch_openvins(self):
        if self.process is not None:
            return
        if not self.config_path:
            self.get_logger().error('Cannot launch OpenVINS: config_path is empty.')
            return

        command = [
            'ros2',
            'run',
            'ov_msckf',
            'run_subscribe_msckf',
            '--ros-args',
            '-r',
            '__node:=%s' % self.node_name,
            '-p',
            'use_sim_time:=true',
            '-p',
            'config_path:=%s' % self.config_path,
            '-p',
            'verbosity:=%s' % self.verbosity,
            '-p',
            'topic_imu:=%s' % self.topic_imu,
            '-p',
            'topic_camera0:=%s' % self.topic_camera0,
            '-p',
            'topic_range:=%s' % self.topic_range,
            '-p',
            'use_stereo:=false',
            '-p',
            'max_cameras:=1',
            '-p',
            'range_feature_enable:=%s' % str(self.range_feature_enable).lower(),
            '-r',
            'odomimu:=/rvio/odometry',
            '-r',
            'poseimu:=/rvio/pose',
            '-r',
            'pathimu:=/rvio/path',
            '-r',
            'trackhist:=/rvio/debug_image',
            '-r',
            'points_slam:=/rvio/points_slam',
            '-r',
            'points_msckf:=/rvio/points_msckf',
        ]
        try:
            self.process = subprocess.Popen(command)
        except OSError as exc:
            self.get_logger().error('Failed to launch OpenVINS: %s' % exc)
            self.process = None

    def destroy_node(self):
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OpenVinsOnStateLauncher()
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
