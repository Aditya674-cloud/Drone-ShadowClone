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


class WaypointDrone(Node):

    def __init__(self):
        super().__init__("waypoint_drone")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.mode_pub = self.create_publisher(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            qos
        )

        self.sp_pub = self.create_publisher(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            qos
        )

        self.cmd_pub = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            qos
        )

        self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v4",
            self.status_cb,
            qos
        )

        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.position_cb,
            qos
        )

        self.current_pos = None
        self.start_pos = None

        self.nav_state = None
        self.arming_state = None
        self.preflight = False

        self.started = time.monotonic()
        self.last_report = 0.0

        self.mode_sent = False
        self.arm_sent = False

        self.waypoint_index = 0
        self.waypoint_reached_time = None
        self.landing_sent = False

        self.timer = self.create_timer(0.05, self.tick)

        self.get_logger().info(
            "Autonomous waypoint controller started."
        )

    def now_us(self):
        return self.get_clock().now().nanoseconds // 1000

    def position_cb(self, msg):
        if not (msg.xy_valid and msg.z_valid):
            return

        if not all(math.isfinite(v) for v in (msg.x, msg.y, msg.z)):
            return

        self.current_pos = [msg.x, msg.y, msg.z]

        if self.start_pos is None:
            self.start_pos = [msg.x, msg.y, msg.z]

            self.get_logger().info(
                f"Start position captured: "
                f"X={msg.x:.2f}, Y={msg.y:.2f}, Z={msg.z:.2f}"
            )

    def status_cb(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
        self.preflight = msg.pre_flight_checks_pass

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

    def get_waypoints(self):

        x0, y0, z0 = self.start_pos

        altitude = z0 - 5.0

        return [
            [x0,     y0,     altitude],
            [x0 + 5, y0,     altitude],
            [x0 + 5, y0 + 5, altitude],
            [x0,     y0 + 5, altitude],
        ]

    def distance_to_waypoint(self, waypoint):

        dx = self.current_pos[0] - waypoint[0]
        dy = self.current_pos[1] - waypoint[1]
        dz = self.current_pos[2] - waypoint[2]

        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def tick(self):

        now_us = self.now_us()
        now = time.monotonic()

        # -------------------------------------------------
        # 1. Publish Offboard heartbeat
        # -------------------------------------------------

        heartbeat = OffboardControlMode()

        heartbeat.timestamp = now_us
        heartbeat.position = True

        heartbeat.velocity = False
        heartbeat.acceleration = False
        heartbeat.attitude = False
        heartbeat.body_rate = False
        heartbeat.thrust_and_torque = False
        heartbeat.direct_actuator = False

        self.mode_pub.publish(heartbeat)

        # -------------------------------------------------
        # 2. Wait until PX4 position is available
        # -------------------------------------------------

        if self.start_pos is None:
            return

        # -------------------------------------------------
        # 3. Generate waypoints
        # -------------------------------------------------

        waypoints = self.get_waypoints()

        # -------------------------------------------------
        # 4. Landing phase
        # -------------------------------------------------

        if self.waypoint_index >= len(waypoints):

            if not self.landing_sent:
                self.get_logger().info(
                    "All waypoints completed. Landing..."
                )
                self.command(21)
                self.landing_sent = True

            return

        # -------------------------------------------------
        # 5. Current target
        # -------------------------------------------------

        target = waypoints[self.waypoint_index]

        # -------------------------------------------------
        # 6. Publish position setpoint
        # -------------------------------------------------

        sp = TrajectorySetpoint()

        sp.timestamp = now_us

        sp.position = target

        sp.velocity = [float("nan")] * 3
        sp.acceleration = [float("nan")] * 3
        sp.jerk = [float("nan")] * 3

        sp.yaw = float("nan")
        sp.yawspeed = float("nan")

        self.sp_pub.publish(sp)

        # -------------------------------------------------
        # 7. Request Offboard mode
        # -------------------------------------------------

        if (
            now - self.started >= 2.0
            and self.nav_state != 14
            and not self.mode_sent
        ):

            self.command(176, 1.0, 6.0)

            self.mode_sent = True

            self.get_logger().info(
                "Offboard mode request sent."
            )

        # -------------------------------------------------
        # 8. Arm
        # -------------------------------------------------

        if (
            self.nav_state == 14
            and self.arming_state == 1
            and self.preflight
            and not self.arm_sent
        ):

            self.command(400, 1.0)

            self.arm_sent = True

            self.get_logger().info(
                "Arm request sent."
            )

        # -------------------------------------------------
        # 9. Check waypoint distance
        # -------------------------------------------------

        if (
            self.arming_state == 2
            and self.nav_state == 14
            and self.current_pos is not None
        ):

            distance = self.distance_to_waypoint(target)

            if distance < 0.5:

                if self.waypoint_reached_time is None:

                    self.waypoint_reached_time = now

                    self.get_logger().info(
                        f"Waypoint {self.waypoint_index + 1} reached."
                    )

                elif now - self.waypoint_reached_time > 2.0:

                    self.waypoint_index += 1

                    self.waypoint_reached_time = None

                    if self.waypoint_index < len(waypoints):

                        self.get_logger().info(
                            f"Moving to waypoint "
                            f"{self.waypoint_index + 1}"
                        )

            else:

                self.waypoint_reached_time = None

                if now - self.last_report > 1.0:

                    self.get_logger().info(
                        f"WP {self.waypoint_index + 1} | "
                        f"Distance: {distance:.2f} m | "
                        f"Position: "
                        f"({self.current_pos[0]:.2f}, "
                        f"{self.current_pos[1]:.2f}, "
                        f"{self.current_pos[2]:.2f})"
                    )

                    self.last_report = now


def main(args=None):

    rclpy.init(args=args)

    node = WaypointDrone()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
