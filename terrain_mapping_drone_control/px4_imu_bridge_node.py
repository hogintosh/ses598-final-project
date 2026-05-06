#!/usr/bin/env python3

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import rclpy

from px4_msgs.msg import SensorCombined
from sensor_msgs.msg import Image, Imu


class Px4ImuBridge(Node):
    """Convert PX4 SensorCombined samples into the sensor_msgs/Imu format OpenVINS expects."""

    def __init__(self):
        super().__init__('px4_imu_bridge')
        self.declare_parameter('px4_sensor_combined_topic', '/fmu/out/sensor_combined')
        self.declare_parameter('imu_topic', '/openvins/imu')
        self.declare_parameter('time_reference_image_topic', '/drone/down_mono')
        self.declare_parameter('frame_id', 'imu')

        px4_topic = str(self.get_parameter('px4_sensor_combined_topic').value)
        imu_topic = str(self.get_parameter('imu_topic').value)
        image_topic = str(self.get_parameter('time_reference_image_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.latest_px4_timestamp_us = None
        self.latest_image_time_sec = None
        self.px4_to_image_time_offset_sec = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(Imu, imu_topic, sensor_qos)
        self.create_subscription(SensorCombined, px4_topic, self.sensor_cb, sensor_qos)
        self.create_subscription(Image, image_topic, self.image_cb, sensor_qos)
        self.get_logger().info(f'PX4 IMU bridge: {px4_topic} -> {imu_topic}, time reference {image_topic}')

    def image_cb(self, msg):
        self.latest_image_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1.0e-9
        if self.latest_px4_timestamp_us is not None:
            self.px4_to_image_time_offset_sec = (
                self.latest_image_time_sec - self.latest_px4_timestamp_us * 1.0e-6
            )

    def sensor_cb(self, msg):
        self.latest_px4_timestamp_us = int(msg.timestamp)
        if self.px4_to_image_time_offset_sec is None:
            return
        out = Imu()
        # PX4 SensorCombined timestamps are synchronized to wall epoch in this
        # SITL setup, while Gazebo camera/range messages use sim time. Re-anchor
        # the PX4 clock on every camera frame so long pre-survey waits and
        # lockstep pauses do not accumulate a fixed IMU/camera offset.
        stamp_sec_float = (
            self.latest_px4_timestamp_us * 1.0e-6
            + self.px4_to_image_time_offset_sec
        )
        stamp_sec = int(stamp_sec_float)
        stamp_nsec = int((stamp_sec_float - stamp_sec) * 1.0e9)
        out.header.stamp.sec = stamp_sec
        out.header.stamp.nanosec = stamp_nsec
        out.header.frame_id = self.frame_id

        out.orientation_covariance[0] = -1.0
        out.angular_velocity.x = float(msg.gyro_rad[0])
        out.angular_velocity.y = float(msg.gyro_rad[1])
        out.angular_velocity.z = float(msg.gyro_rad[2])
        out.linear_acceleration.x = float(msg.accelerometer_m_s2[0])
        out.linear_acceleration.y = float(msg.accelerometer_m_s2[1])
        out.linear_acceleration.z = float(msg.accelerometer_m_s2[2])

        out.angular_velocity_covariance[0] = 1.0e-4
        out.angular_velocity_covariance[4] = 1.0e-4
        out.angular_velocity_covariance[8] = 1.0e-4
        out.linear_acceleration_covariance[0] = 1.0e-3
        out.linear_acceleration_covariance[4] = 1.0e-3
        out.linear_acceleration_covariance[8] = 1.0e-3
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = Px4ImuBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
