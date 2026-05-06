#!/usr/bin/env python3

import subprocess

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32
from std_msgs.msg import String


class RvioRqtPlotLauncher(Node):
    """Open RVIO rqt_plot windows when the mission enters the plotting phase."""

    def __init__(self):
        super().__init__('rvio_rqt_plot_launcher')

        self.declare_parameter('mission_state_topic', '/mission/state')
        self.declare_parameter('open_on_state', 'SURVEY')
        self.declare_parameter('open_x_plot', True)
        self.declare_parameter('open_y_plot', True)
        self.declare_parameter('open_debug_image', True)
        self.declare_parameter('plot_wait_timeout_sec', 0.0)

        self.mission_state_topic = str(
            self.get_parameter('mission_state_topic').value
        )
        self.open_on_state = str(self.get_parameter('open_on_state').value).upper()
        self.open_x_plot = bool(self.get_parameter('open_x_plot').value)
        self.open_y_plot = bool(self.get_parameter('open_y_plot').value)
        self.open_debug_image = bool(self.get_parameter('open_debug_image').value)
        self.plot_wait_timeout_sec = float(
            self.get_parameter('plot_wait_timeout_sec').value
        )

        self.opened = False
        self.plot_requested = False
        self.plot_request_time = None
        self.processes = []
        self.seen_plot_topics = {
            '/rvio_plot/ground_truth_x': False,
            '/rvio_plot/estimated_x': False,
            '/rvio_plot/ground_truth_y': False,
            '/rvio_plot/estimated_y': False,
        }

        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            String, self.mission_state_topic, self.mission_state_cb, state_qos
        )
        for topic in self.seen_plot_topics:
            self.create_subscription(
                Float32,
                topic,
                lambda _msg, topic=topic: self.plot_sample_cb(topic),
                10,
            )
        self.create_timer(0.25, self.maybe_open_plots)

        self.get_logger().info(
            'RVIO rqt plot launcher waiting for %s on %s.'
            % (self.open_on_state, self.mission_state_topic)
        )

    def mission_state_cb(self, msg):
        state = str(msg.data).upper()
        if self.opened or self.plot_requested or state != self.open_on_state:
            return

        self.plot_requested = True
        self.plot_request_time = self.get_clock().now()
        self.get_logger().info(
            'Mission entered %s; waiting for first RVIO plot samples.' % state
        )

    def plot_sample_cb(self, topic):
        self.seen_plot_topics[topic] = True

    def maybe_open_plots(self):
        if not self.plot_requested or self.opened:
            return

        needed_topics = []
        if self.open_x_plot:
            needed_topics.extend([
                '/rvio_plot/ground_truth_x',
                '/rvio_plot/estimated_x',
            ])
        if self.open_y_plot:
            needed_topics.extend([
                '/rvio_plot/ground_truth_y',
                '/rvio_plot/estimated_y',
            ])

        missing_topics = [
            topic for topic in needed_topics if not self.seen_plot_topics[topic]
        ]
        if missing_topics:
            if self.plot_wait_timeout_sec <= 0.0:
                return
            elapsed = (
                self.get_clock().now() - self.plot_request_time
            ).nanoseconds * 1e-9
            if elapsed < self.plot_wait_timeout_sec:
                return
            self.get_logger().warn(
                'Opening RVIO rqt plots before all samples arrived; missing %s'
                % ', '.join(missing_topics)
            )

        self.opened = True
        self.get_logger().info('Opening RVIO rqt plots with live samples.')
        if self.open_x_plot:
            self._spawn([
                'ros2',
                'run',
                'terrain_mapping_drone_control',
                'rvio_rqt_plot_retry',
                '--clear-config',
                '--force-discover',
                '--empty',
                '/rvio_plot/ground_truth_x/data',
                '/rvio_plot/estimated_x/data',
            ])
        if self.open_y_plot:
            self._spawn([
                'ros2',
                'run',
                'terrain_mapping_drone_control',
                'rvio_rqt_plot_retry',
                '--clear-config',
                '--force-discover',
                '--empty',
                '/rvio_plot/ground_truth_y/data',
                '/rvio_plot/estimated_y/data',
            ])
        if self.open_debug_image:
            self._spawn([
                '/opt/ros/jazzy/lib/rqt_image_view/rqt_image_view',
                '/rvio/debug_image',
            ])

    def _spawn(self, command):
        try:
            self.processes.append(subprocess.Popen(command))
        except OSError as exc:
            self.get_logger().error('Failed to launch %s: %s' % (' '.join(command), exc))

    def destroy_node(self):
        for process in self.processes:
            if process.poll() is None:
                process.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RvioRqtPlotLauncher()
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
