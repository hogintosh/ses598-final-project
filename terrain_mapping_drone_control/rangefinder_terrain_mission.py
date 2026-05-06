#!/usr/bin/env python3

import math
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    BatteryStatus,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleOdometry,
    VehicleStatus,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String


class RangefinderTerrainMission(Node):
    """Terrain-relative survey mission using a downward 1-DOF lidar."""

    def __init__(self):
        super().__init__('rangefinder_terrain_mission')

        self.declare_parameter('takeoff_agl', 5.0)
        self.declare_parameter('mission_mode', 'auto_hover')
        self.declare_parameter('hover_relative_height', 2.0)
        self.declare_parameter('require_preflight_checks', True)
        self.declare_parameter('bootstrap_takeoff_height', 1.5)
        self.declare_parameter('bootstrap_takeoff_tolerance', 0.25)
        self.declare_parameter('survey_agl', 5.0)
        self.declare_parameter('survey_speed', 2.0)
        self.declare_parameter('waypoint_tolerance', 0.6)
        self.declare_parameter('rvio_test_agl', 5.0)
        self.declare_parameter('rvio_test_climb_height', 5.0)
        self.declare_parameter('rvio_test_motion_profile', 'single_axis')
        self.declare_parameter(
            'rvio_test_axis',
            'x',
            ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter('rvio_test_axis_distance', 15.0)
        self.declare_parameter('rvio_test_axis_speed', 0.5)
        self.declare_parameter(
            'rvio_test_waypoints',
            [
                0.0, 0.0,
                5.0, 0.0,
                5.0, 3.0,
                0.0, 3.0,
                0.0, 0.0,
            ],
        )
        self.declare_parameter('range_timeout_sec', 1.0)
        self.declare_parameter('ground_filter_alpha', 0.25)
        self.declare_parameter('landing_agl', 0.8)
        self.declare_parameter('land_command_agl', 0.35)
        self.declare_parameter(
            'survey_waypoints',
            [
                0.0, 0.0,
                8.0, 0.0,
                8.0, 4.0,
                0.0, 4.0,
                0.0, 8.0,
                8.0, 8.0,
            ],
        )

        self.takeoff_agl = float(self.get_parameter('takeoff_agl').value)
        self.mission_mode = str(self.get_parameter('mission_mode').value)
        self.hover_relative_height = float(
            self.get_parameter('hover_relative_height').value
        )
        self.require_preflight_checks = bool(
            self.get_parameter('require_preflight_checks').value
        )
        self.bootstrap_takeoff_height = float(
            self.get_parameter('bootstrap_takeoff_height').value
        )
        self.bootstrap_takeoff_tolerance = float(
            self.get_parameter('bootstrap_takeoff_tolerance').value
        )
        self.survey_agl = float(self.get_parameter('survey_agl').value)
        self.survey_speed = float(self.get_parameter('survey_speed').value)
        self.waypoint_tolerance = float(self.get_parameter('waypoint_tolerance').value)
        self.rvio_test_agl = float(self.get_parameter('rvio_test_agl').value)
        self.rvio_test_climb_height = float(
            self.get_parameter('rvio_test_climb_height').value
        )
        self.rvio_test_motion_profile = str(
            self.get_parameter('rvio_test_motion_profile').value
        ).strip().lower()
        self.rvio_test_axis = self._axis_parameter_value(
            self.get_parameter('rvio_test_axis').value
        )
        self.rvio_test_axis_distance = float(
            self.get_parameter('rvio_test_axis_distance').value
        )
        self.rvio_test_axis_speed = max(
            0.05,
            abs(float(self.get_parameter('rvio_test_axis_speed').value)),
        )
        self.range_timeout_sec = float(self.get_parameter('range_timeout_sec').value)
        self.ground_filter_alpha = float(self.get_parameter('ground_filter_alpha').value)
        self.landing_agl = float(self.get_parameter('landing_agl').value)
        self.land_command_agl = float(self.get_parameter('land_command_agl').value)
        self.survey_waypoints = self._parse_waypoints(
            self.get_parameter('survey_waypoints').value
        )
        if self.mission_mode == 'rvio_test':
            self.takeoff_agl = self.rvio_test_agl
            self.survey_agl = self.rvio_test_agl
            if self.rvio_test_motion_profile != 'single_axis':
                self.survey_waypoints = self._parse_waypoints(
                    self.get_parameter('rvio_test_waypoints').value
                )

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_qos
        )
        self.vehicle_cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', px4_qos
        )
        self.agl_pub = self.create_publisher(Float32, '/mission/agl_estimate', 10)
        self.ground_pub = self.create_publisher(Float32, '/mission/ground_z_ned', 10)
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.state_pub = self.create_publisher(String, '/mission/state', state_qos)

        self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_cb, px4_qos
        )
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_position_cb,
            px4_qos,
        )
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', self.status_cb, px4_qos
        )
        self.create_subscription(
            BatteryStatus, '/fmu/out/battery_status', self.battery_cb, px4_qos
        )
        self.create_subscription(
            LaserScan, '/drone/down_rangefinder', self.rangefinder_cb, 10
        )

        self.position = [0.0, 0.0, 0.0]
        self.odom_received = False
        self.vehicle_status = VehicleStatus()
        self.battery_percent = None
        self.initial_battery = None
        self.final_battery = None

        self.agl = None
        self.last_range_time = None
        self.filtered_ground_z = None
        self.offboard_setpoint_counter = 0
        self.state = 'WAIT_ODOM'
        self.start_time = None
        self.hover_target = None
        self.rvio_test_target_z = None
        self.rvio_single_axis_start_time = None
        self.rvio_single_axis_start = None
        self.rvio_single_axis_end = None
        self.active_waypoint = 0
        self.landing_command_sent = False

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info(
            'Rangefinder terrain mission ready. Waiting for PX4 odometry before '
            f'{self.mission_mode}.'
        )

    def _axis_parameter_value(self, value):
        if isinstance(value, bool):
            return 'y' if value else 'x'
        axis = str(value).strip().lower()
        if axis in ('true', 'yes'):
            return 'y'
        if axis in ('false', 'no'):
            return 'x'
        return axis

    def _parse_waypoints(self, flat_values):
        values = [float(v) for v in flat_values]
        if len(values) < 2 or len(values) % 2 != 0:
            raise ValueError('survey_waypoints must contain x/y pairs')
        return [(values[i], values[i + 1]) for i in range(0, len(values), 2)]

    def configure_rvio_test_waypoints(self):
        if self.rvio_test_motion_profile != 'single_axis':
            return

        axis = self.rvio_test_axis
        if axis not in ('x', 'y'):
            self.get_logger().warn(
                f'Unsupported rvio_test_axis={axis!r}; using x.'
            )
            axis = 'x'

        start_x = float(self.position[0])
        start_y = float(self.position[1])
        distance = float(self.rvio_test_axis_distance)
        end_x = start_x + (distance if axis == 'x' else 0.0)
        end_y = start_y + (distance if axis == 'y' else 0.0)
        self.rvio_single_axis_start_time = None
        self.rvio_single_axis_start = (start_x, start_y)
        self.rvio_single_axis_end = (end_x, end_y)
        self.survey_waypoints = [(start_x, start_y), (end_x, end_y)]
        self.get_logger().info(
            'RVIO single-axis survey configured: axis=%s distance=%.2f m '
            'speed=%.2f m/s from (%.2f, %.2f) to (%.2f, %.2f).'
            % (
                axis,
                distance,
                self.rvio_test_axis_speed,
                start_x,
                start_y,
                end_x,
                end_y,
            )
        )

    def run_rvio_single_axis_survey(self):
        if self.rvio_single_axis_start is None or self.rvio_single_axis_end is None:
            self.configure_rvio_test_waypoints()
        if self.rvio_single_axis_start is None or self.rvio_single_axis_end is None:
            self.hold_current_position()
            return

        if self.rvio_single_axis_start_time is None:
            self.rvio_single_axis_start_time = time.time()

        start_x, start_y = self.rvio_single_axis_start
        end_x, end_y = self.rvio_single_axis_end
        dx = end_x - start_x
        dy = end_y - start_y
        distance = math.hypot(dx, dy)
        if distance <= 1.0e-6:
            self.state = 'RETURN'
            self.get_logger().info('Single-axis distance is zero. Returning to launch.')
            return

        elapsed = max(0.0, time.time() - self.rvio_single_axis_start_time)
        progress = min(distance, elapsed * self.rvio_test_axis_speed)
        ratio = progress / distance
        x = start_x + ratio * dx
        y = start_y + ratio * dy
        z = self.survey_z_target()
        yaw = math.atan2(dy, dx)
        if progress < distance:
            vx = (dx / distance) * self.rvio_test_axis_speed
            vy = (dy / distance) * self.rvio_test_axis_speed
        else:
            vx = 0.0
            vy = 0.0
        self.publish_trajectory_setpoint(x, y, z, yaw, velocity=(vx, vy, 0.0))

        if progress >= distance and self.xy_distance_to(end_x, end_y) < self.waypoint_tolerance:
            self.state = 'RETURN'
            self.get_logger().info('RVIO single-axis survey complete. Returning to launch for landing.')

    def odom_cb(self, msg):
        self.position = [float(msg.position[0]), float(msg.position[1]), float(msg.position[2])]
        self.odom_received = True

    def local_position_cb(self, msg):
        if not (msg.xy_valid and msg.z_valid):
            return
        self.position = [float(msg.x), float(msg.y), float(msg.z)]
        self.odom_received = True

    def status_cb(self, msg):
        self.vehicle_status = msg

    def battery_cb(self, msg):
        if not math.isnan(msg.volt_based_soc_estimate):
            self.battery_percent = float(msg.volt_based_soc_estimate)

    def rangefinder_cb(self, msg):
        valid_ranges = [
            r for r in msg.ranges
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max
        ]
        if not valid_ranges:
            return

        self.agl = min(valid_ranges)
        self.last_range_time = time.time()
        measured_ground_z = self.position[2] + self.agl

        if self.filtered_ground_z is None:
            self.filtered_ground_z = measured_ground_z
        else:
            alpha = self.ground_filter_alpha
            self.filtered_ground_z = (
                alpha * measured_ground_z + (1.0 - alpha) * self.filtered_ground_z
            )

        agl_msg = Float32()
        agl_msg.data = float(self.agl)
        self.agl_pub.publish(agl_msg)

        ground_msg = Float32()
        ground_msg.data = float(self.filtered_ground_z)
        self.ground_pub.publish(ground_msg)

    def control_loop(self):
        self.publish_mission_state()
        self.publish_offboard_control_mode()
        self.offboard_setpoint_counter += 1

        if self.state == 'WAIT_ODOM':
            if not self.odom_received:
                return
            self.state = 'BOOTSTRAP_TAKEOFF'
            self.offboard_setpoint_counter = 0
            self.start_time = time.time()
            if self.battery_percent is not None:
                self.initial_battery = self.battery_percent
            if self.mission_mode == 'auto_hover':
                self.hover_target = (
                    self.position[0],
                    self.position[1],
                    self.position[2] - abs(self.hover_relative_height),
                    0.0,
                )
                self.state = 'AUTO_HOVER'
                self.get_logger().info(
                    'PX4 odometry locked. Auto-hover target set to '
                    f'x={self.hover_target[0]:.2f}, y={self.hover_target[1]:.2f}, '
                    f'z={self.hover_target[2]:.2f} NED.'
                )
                return
            if self.mission_mode == 'rvio_test':
                self.rvio_test_target_z = (
                    self.position[2] - abs(self.rvio_test_climb_height)
                )
                self.state = 'RVIO_TEST_CLIMB'
                self.get_logger().info(
                    'PX4 odometry locked. RVIO test climbing %.1f m to '
                    'z=%.2f NED before starting metrics.'
                    % (abs(self.rvio_test_climb_height), self.rvio_test_target_z)
                )
                return
            self.get_logger().info(
                'PX4 odometry locked. Bootstrapping to '
                f'{self.bootstrap_takeoff_height:.1f} m before requiring lidar.'
            )

        elif self.state == 'AUTO_HOVER':
            self.run_auto_hover()

        elif self.state == 'RVIO_TEST_CLIMB':
            if self.rvio_test_target_z is None:
                self.rvio_test_target_z = (
                    self.position[2] - abs(self.rvio_test_climb_height)
                )

            if 5 <= self.offboard_setpoint_counter <= 50:
                self.engage_offboard_mode()
                self.arm()

            self.publish_trajectory_setpoint(
                self.position[0], self.position[1], self.rvio_test_target_z, 0.0
            )

            if abs(self.position[2] - self.rvio_test_target_z) < self.bootstrap_takeoff_tolerance:
                self.state = 'SURVEY'
                self.offboard_setpoint_counter = 0
                self.active_waypoint = 0
                self.configure_rvio_test_waypoints()
                self.get_logger().info(
                    'RVIO test climb complete. Beginning horizontal survey and metrics.'
                )

        elif self.state == 'BOOTSTRAP_TAKEOFF':
            if 5 <= self.offboard_setpoint_counter <= 50:
                self.engage_offboard_mode()
                self.arm()

            target_z = -abs(self.bootstrap_takeoff_height)
            self.publish_trajectory_setpoint(0.0, 0.0, target_z, 0.0)

            if abs(self.position[2] - target_z) < self.bootstrap_takeoff_tolerance:
                self.state = 'WAIT_RANGE'
                self.offboard_setpoint_counter = 0
                self.get_logger().info(
                    'Bootstrap altitude reached. Waiting for finite downward lidar.'
                )

        elif self.state == 'WAIT_RANGE':
            self.hold_current_position()
            if not self.range_is_fresh():
                if self.offboard_setpoint_counter % 10 == 0:
                    self.get_logger().warn(
                        'Waiting for finite rangefinder data before terrain-relative climb.'
                    )
                return
            self.state = 'ARM_TAKEOFF'
            self.offboard_setpoint_counter = 0
            self.get_logger().info('Rangefinder locked. Starting takeoff sequence.')

        elif self.state == 'ARM_TAKEOFF':
            if not self.range_is_fresh():
                self.hold_current_position()
                if self.offboard_setpoint_counter % 10 == 0:
                    self.get_logger().warn('Rangefinder data is stale; holding position.')
                return

            if 5 <= self.offboard_setpoint_counter <= 50:
                self.engage_offboard_mode()
                self.arm()

            target_z = self.target_z_for_agl(self.takeoff_agl)
            self.publish_trajectory_setpoint(0.0, 0.0, target_z, 0.0)
            if self.agl is not None and abs(self.agl - self.takeoff_agl) < 0.4:
                self.state = 'SURVEY'
                self.active_waypoint = 0
                self.get_logger().info('Takeoff AGL reached. Beginning terrain-relative survey.')

        elif self.state == 'SURVEY':
            if self.mission_mode != 'rvio_test' and not self.range_is_fresh():
                self.hold_current_position()
                if self.offboard_setpoint_counter % 10 == 0:
                    self.get_logger().warn('Rangefinder data is stale; holding position.')
                return

            if (
                self.mission_mode == 'rvio_test'
                and self.rvio_test_motion_profile == 'single_axis'
            ):
                self.run_rvio_single_axis_survey()
                return

            x, y = self.survey_waypoints[self.active_waypoint]
            z = self.survey_z_target()
            yaw = math.atan2(y - self.position[1], x - self.position[0])
            self.publish_trajectory_setpoint(x, y, z, yaw)

            if self.xy_distance_to(x, y) < self.waypoint_tolerance:
                self.active_waypoint += 1
                if self.active_waypoint >= len(self.survey_waypoints):
                    self.state = 'RETURN'
                    self.get_logger().info('Survey complete. Returning to launch for landing.')
                else:
                    self.get_logger().info(
                        f'Advancing to survey waypoint {self.active_waypoint + 1}/'
                        f'{len(self.survey_waypoints)}.'
                    )

        elif self.state == 'RETURN':
            if self.mission_mode != 'rvio_test' and not self.range_is_fresh():
                self.hold_current_position()
                if self.offboard_setpoint_counter % 10 == 0:
                    self.get_logger().warn('Rangefinder data is stale; holding position.')
                return

            z = self.survey_z_target()
            self.publish_trajectory_setpoint(0.0, 0.0, z, 0.0)
            if self.xy_distance_to(0.0, 0.0) < self.waypoint_tolerance:
                self.state = 'DESCEND'
                self.get_logger().info('Back at launch point. Starting lidar-guided descent.')

        elif self.state == 'DESCEND':
            if not self.range_is_fresh():
                self.hold_current_position()
                if self.offboard_setpoint_counter % 10 == 0:
                    self.get_logger().warn('Rangefinder data is stale; holding position.')
                return

            z = self.target_z_for_agl(self.landing_agl)
            self.publish_trajectory_setpoint(0.0, 0.0, z, 0.0)
            if self.agl is not None and self.agl <= self.land_command_agl:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.landing_command_sent = True
                self.state = 'LAND'
                self.get_logger().info('Near ground by rangefinder. LAND command sent.')

        elif self.state == 'LAND':
            if self.agl is not None and self.agl <= 0.15:
                self.disarm()
                self.state = 'COMPLETE'

        elif self.state == 'COMPLETE':
            if self.final_battery is None and self.battery_percent is not None:
                self.final_battery = self.battery_percent
            self.log_mission_summary()
            self.state = 'DONE'

    def publish_mission_state(self):
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

    def range_is_fresh(self):
        return (
            self.last_range_time is not None
            and time.time() - self.last_range_time <= self.range_timeout_sec
            and self.filtered_ground_z is not None
        )

    def target_z_for_agl(self, desired_agl):
        return float(self.filtered_ground_z - desired_agl)

    def survey_z_target(self):
        if self.mission_mode == 'rvio_test' and self.rvio_test_target_z is not None:
            return float(self.rvio_test_target_z)
        return self.target_z_for_agl(self.survey_agl)

    def xy_distance_to(self, x, y):
        return math.hypot(self.position[0] - x, self.position[1] - y)

    def hold_current_position(self):
        self.publish_trajectory_setpoint(
            self.position[0], self.position[1], self.position[2], 0.0
        )

    def run_auto_hover(self):
        if self.hover_target is None:
            self.hover_target = (
                self.position[0],
                self.position[1],
                self.position[2] - abs(self.hover_relative_height),
                0.0,
            )

        x, y, z, yaw = self.hover_target
        self.publish_trajectory_setpoint(x, y, z, yaw)

        checks_ready = (
            self.vehicle_status.pre_flight_checks_pass
            or not self.require_preflight_checks
        )
        if not checks_ready:
            if self.offboard_setpoint_counter % 20 == 0:
                self.get_logger().warn(
                    'Waiting for PX4 preflight checks before auto-arm hover.'
                )
            return

        if self.offboard_setpoint_counter >= 10:
            if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.engage_offboard_mode()
            if self.vehicle_status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                self.arm()

        if self.offboard_setpoint_counter % 30 == 0:
            armed = self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED
            offboard = (
                self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            )
            self.get_logger().info(
                f'Auto-hover holding target z={z:.2f} NED '
                f'(armed={armed}, offboard={offboard}).'
            )

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z, yaw, velocity=None):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        if velocity is None:
            msg.velocity = [math.nan, math.nan, math.nan]
        else:
            msg.velocity = [float(velocity[0]), float(velocity[1]), float(velocity[2])]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.jerk = [math.nan, math.nan, math.nan]
        msg.yaw = float(yaw)
        msg.yawspeed = math.nan
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.trajectory_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = int(command)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.vehicle_cmd_pub.publish(msg)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)

    def engage_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0
        )

    def log_mission_summary(self):
        duration = time.time() - self.start_time if self.start_time is not None else 0.0
        self.get_logger().info(f'Mission complete in {duration:.1f}s.')
        if self.initial_battery is not None and self.final_battery is not None:
            used = (self.initial_battery - self.final_battery) * 100.0
            self.get_logger().info(f'Battery used: {used:.3f}%')


def main(args=None):
    rclpy.init(args=args)
    node = RangefinderTerrainMission()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
