import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition,
)
from sensor_msgs.msg import LaserScan


SCAN_TOPIC = (
    '/world/shadowclone_obstacle/model/x500_lidar_2d_0/'
    'link/link/sensor/lidar_2d_v2/scan'
)


class ObstacleAvoidanceDrone(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance_drone')

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            px4_qos
        )

        self.sp_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            px4_qos
        )

        self.cmd_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            px4_qos
        )

        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v4',
            self.status_cb,
            px4_qos
        )

        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.position_cb,
            px4_qos
        )

        self.create_subscription(
            LaserScan,
            SCAN_TOPIC,
            self.scan_cb,
            scan_qos
        )

        self.position = None
        self.start_z = None
        self.heading = 0.0

        self.nav_state = None
        self.arming_state = None
        self.preflight = False

        self.front_distance = None
        self.last_scan_time = None

        self.started = time.monotonic()
        self.mode_sent = False
        self.arm_sent = False

        self.phase = 'TAKEOFF'
        self.stop_started = None
        self.landing_sent = False
        self.last_report = 0.0

        self.takeoff_height = 2.0
        self.approach_speed = 0.5
        self.stop_distance = 1.5

        self.timer = self.create_timer(0.05, self.tick)

        self.get_logger().info(
            'ShadowClone obstacle-response controller started.'
        )

    def now_us(self):
        return self.get_clock().now().nanoseconds // 1000

    def position_cb(self, msg):

        if not (msg.xy_valid and msg.z_valid):
            return

        if not all(math.isfinite(v) for v in (msg.x, msg.y, msg.z)):
            return

        self.position = [msg.x, msg.y, msg.z]
        self.heading = msg.heading

        if self.start_z is None:
            self.start_z = msg.z

            self.get_logger().info(
                f'Initial position captured: Z={msg.z:.2f}'
            )

    def status_cb(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
        self.preflight = msg.pre_flight_checks_pass

    def scan_cb(self, scan):

        readings = []

        for i, distance in enumerate(scan.ranges):

            angle = scan.angle_min + i * scan.angle_increment
            angle_deg = math.degrees(angle)

            # Forward sector: 15 degrees left to 15 degrees right.
            if abs(angle_deg) > 15.0:
                continue

            if not math.isfinite(distance):
                continue

            if not scan.range_min <= distance <= scan.range_max:
                continue

            readings.append(distance)

        self.front_distance = min(readings) if readings else None
        self.last_scan_time = time.monotonic()

    def command(self, command_id, p1=0.0, p2=0.0):

        msg = VehicleCommand()

        msg.timestamp = self.now_us()
        msg.command = command_id
        msg.param1 = float(p1)
        msg.param2 = float(p2)

        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.cmd_pub.publish(msg)

    def publish_setpoint(self, vx, vy, vz):

        now_us = self.now_us()

        heartbeat = OffboardControlMode()

        heartbeat.timestamp = now_us
        heartbeat.position = False
        heartbeat.velocity = True
        heartbeat.acceleration = False
        heartbeat.attitude = False
        heartbeat.body_rate = False
        heartbeat.thrust_and_torque = False
        heartbeat.direct_actuator = False

        self.mode_pub.publish(heartbeat)

        sp = TrajectorySetpoint()

        sp.timestamp = now_us

        sp.position = [float('nan')] * 3
        sp.velocity = [
            float(vx),
            float(vy),
            float(vz)
        ]
        sp.acceleration = [float('nan')] * 3
        sp.jerk = [float('nan')] * 3

        sp.yaw = float(self.heading)
        sp.yawspeed = 0.0

        self.sp_pub.publish(sp)

    def land(self):

        if not self.landing_sent:
            self.get_logger().warning(
                'Landing command sent.'
            )

            self.command(21)

            self.landing_sent = True

    def tick(self):

        now = time.monotonic()

        # Wait for valid position.
        if self.position is None or self.start_z is None:
            return

        # Desired altitude in NED.
        target_z = self.start_z - self.takeoff_height

        z_error = target_z - self.position[2]

        vz = max(
            -0.5,
            min(0.5, 0.8 * z_error)
        )

        vx = 0.0
        vy = 0.0

        scan_fresh = (
            self.last_scan_time is not None
            and now - self.last_scan_time < 1.0
        )

        # TAKEOFF

        if self.phase == 'TAKEOFF':

            if abs(z_error) < 0.2:

                self.phase = 'APPROACH'

                self.get_logger().info(
                    'Takeoff altitude reached. '
                    'Starting approach.'
                )

        # APPROACH OBSTACLE

        if self.phase == 'APPROACH':

            if not scan_fresh:

                vx = 0.0

                self.get_logger().warning(
                    'LiDAR data stale. '
                    'Forward movement stopped.'
                )

                if (
                    self.last_scan_time is not None
                    and now - self.last_scan_time > 2.0
                ):
                    self.phase = 'LAND'

            elif self.front_distance is None:

                vx = 0.0

                self.get_logger().warning(
                    'No valid forward LiDAR return. '
                    'Holding position.'
                )

            elif self.front_distance <= self.stop_distance:

                self.phase = 'STOP'
                self.stop_started = now

                self.get_logger().warning(
                    f'OBSTACLE DETECTED at '
                    f'{self.front_distance:.2f} m. '
                    f'STOPPING!'
                )

            else:

                # Convert body-forward velocity
                # into local NED velocity.

                vx = (
                    self.approach_speed
                    * math.cos(self.heading)
                )

                vy = (
                    self.approach_speed
                    * math.sin(self.heading)
                )

                if now - self.last_report > 1.0:

                    self.get_logger().info(
                        f'APPROACH | Obstacle: '
                        f'{self.front_distance:.2f} m'
                    )

                    self.last_report = now
                  
        # STOP

        elif self.phase == 'STOP':

            vx = 0.0
            vy = 0.0

            if now - self.stop_started >= 3.0:

                self.phase = 'LAND'

                self.get_logger().info(
                    'Obstacle stop test complete.'
                )

        #  LAND

        elif self.phase == 'LAND':

            vx = 0.0
            vy = 0.0

            self.land()

        # Maintain altitude while not landing.
        if self.phase != 'LAND':

            vz = max(
                -0.5,
                min(0.5, 0.8 * z_error)
            )

        else:

            vz = 0.0

        self.publish_setpoint(
            vx,
            vy,
            vz
        )

        # REQUEST OFFBOARD

        if (
            now - self.started >= 2.0
            and self.nav_state != 14
            and not self.mode_sent
        ):

            self.command(
                176,
                1.0,
                6.0
            )

            self.mode_sent = True

            self.get_logger().info(
                'Offboard mode request sent.'
            )
        # ARM

        if (
            self.nav_state == 14
            and self.arming_state == 1
            and self.preflight
            and not self.arm_sent
        ):

            self.command(
                400,
                1.0
            )

            self.arm_sent = True

            self.get_logger().info(
                'Arm request sent.'
            )


def main(args=None):

    rclpy.init(args=args)

    node = ObstacleAvoidanceDrone()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


