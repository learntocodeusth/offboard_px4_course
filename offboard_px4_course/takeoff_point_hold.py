#!/usr/bin/env python3

from __future__ import annotations

import math
from enum import Enum

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class MissionPhase(str, Enum):
    PREFLIGHT = "PREFLIGHT"
    TAKEOFF = "TAKEOFF"
    TRANSIT = "TRANSIT"
    HOLD = "HOLD"


class TakeoffPointHoldController(Node):
    TARGET_ALT = 2.5
    TARGET_X_OFFSET = 3.0
    TARGET_Y_OFFSET = 0.0
    SETPOINT_PERIOD = 0.1
    REQUEST_PERIOD = 1.0
    PREFLIGHT_SETPOINTS = 20
    ASCENT_RATE = 0.8
    ALTITUDE_TOLERANCE = 0.20
    XY_TOLERANCE = 0.25

    def __init__(self) -> None:
        super().__init__("takeoff_point_hold_controller")

        self.declare_parameter("target_altitude", self.TARGET_ALT)
        self.declare_parameter("target_x_offset", self.TARGET_X_OFFSET)
        self.declare_parameter("target_y_offset", self.TARGET_Y_OFFSET)

        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        sp_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pos_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", sp_qos
        )
        self.arm_cli = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode_cli = self.create_client(SetMode, "/mavros/set_mode")

        self.create_subscription(State, "/mavros/state", self.state_cb, state_qos)
        self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self.pose_cb, pose_qos
        )

        self.target_altitude = (
            self.get_parameter("target_altitude").get_parameter_value().double_value
        )
        self.target_x_offset = (
            self.get_parameter("target_x_offset").get_parameter_value().double_value
        )
        self.target_y_offset = (
            self.get_parameter("target_y_offset").get_parameter_value().double_value
        )

        self.state = State()
        self.pose = PoseStamped()
        self.have_pose = False
        self.phase = MissionPhase.PREFLIGHT
        self.preflight_counter = 0
        self.last_req = self.get_clock().now()
        self.phase_started_at = self.get_clock().now()
        self.home_position = Point()
        self.target_xy = Point()
        self.target_z = 0.1
        self.target_orientation = euler_to_quaternion(0.0, 0.0, 0.0)

        self.create_timer(self.SETPOINT_PERIOD, self.control_loop)
        self.get_logger().info(
            "Takeoff point-hold controller initialised "
            f"(x_offset={self.target_x_offset}, y_offset={self.target_y_offset}, "
            f"alt={self.target_altitude})"
        )

    def state_cb(self, msg: State) -> None:
        self.state = msg

    def pose_cb(self, msg: PoseStamped) -> None:
        self.pose = msg
        if self.have_pose:
            return

        self.have_pose = True
        self.home_position.x = msg.pose.position.x
        self.home_position.y = msg.pose.position.y
        self.home_position.z = msg.pose.position.z
        self.target_xy.x = self.home_position.x
        self.target_xy.y = self.home_position.y
        self.target_z = self.home_position.z + 0.1

        orientation = msg.pose.orientation
        if any(
            abs(value) > 1e-6
            for value in (
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            )
        ):
            self.target_orientation = orientation

        self.get_logger().info(
            "Pose locked at "
            f"x={self.home_position.x:.2f}, "
            f"y={self.home_position.y:.2f}, "
            f"z={self.home_position.z:.2f}"
        )

    def _call(self, cli, req, label: str) -> None:
        if cli.wait_for_service(timeout_sec=1.0):
            cli.call_async(req)
            self.get_logger().info(f"{label} request")
        else:
            self.get_logger().warn(f"{label} service unavailable")

    def arm(self) -> None:
        self._call(self.arm_cli, CommandBool.Request(value=True), "Arm")

    def set_mode(self, mode: str) -> None:
        self._call(self.mode_cli, SetMode.Request(custom_mode=mode), f"Mode {mode}")

    def transition_to(self, phase: MissionPhase) -> None:
        self.phase = phase
        self.phase_started_at = self.get_clock().now()

        if phase == MissionPhase.TAKEOFF:
            self.target_xy.x = self.home_position.x
            self.target_xy.y = self.home_position.y
        elif phase == MissionPhase.TRANSIT:
            self.target_xy.x = self.home_position.x + self.target_x_offset
            self.target_xy.y = self.home_position.y + self.target_y_offset
            self.target_z = self.home_position.z + self.target_altitude
        elif phase == MissionPhase.HOLD:
            self.target_xy.x = self.home_position.x + self.target_x_offset
            self.target_xy.y = self.home_position.y + self.target_y_offset
            self.target_z = self.home_position.z + self.target_altitude

        self.get_logger().info(f"Mission phase -> {phase.value}")

    def seconds_in_phase(self) -> float:
        now = self.get_clock().now()
        return (now - self.phase_started_at).nanoseconds / 1e9

    def altitude_above_home(self) -> float:
        return self.pose.pose.position.z - self.home_position.z

    def distance_to_target_xy(self) -> float:
        dx = self.pose.pose.position.x - self.target_xy.x
        dy = self.pose.pose.position.y - self.target_xy.y
        return math.hypot(dx, dy)

    def build_setpoint(self) -> PoseStamped:
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = "map"
        sp.pose.position.x = self.target_xy.x
        sp.pose.position.y = self.target_xy.y
        sp.pose.position.z = self.target_z
        sp.pose.orientation = self.target_orientation
        return sp

    def update_mission_target(self) -> None:
        if self.phase == MissionPhase.TAKEOFF:
            self.target_xy.x = self.home_position.x
            self.target_xy.y = self.home_position.y
            climb = self.ASCENT_RATE * self.seconds_in_phase()
            self.target_z = min(
                self.home_position.z + self.target_altitude,
                self.home_position.z + 0.1 + climb,
            )
            if self.altitude_above_home() >= (
                self.target_altitude - self.ALTITUDE_TOLERANCE
            ):
                self.target_z = self.home_position.z + self.target_altitude
                self.transition_to(MissionPhase.TRANSIT)
            return

        if self.phase == MissionPhase.TRANSIT:
            self.target_z = self.home_position.z + self.target_altitude
            if self.distance_to_target_xy() <= self.XY_TOLERANCE:
                self.transition_to(MissionPhase.HOLD)
            return

        if self.phase == MissionPhase.HOLD:
            self.target_xy.x = self.home_position.x + self.target_x_offset
            self.target_xy.y = self.home_position.y + self.target_y_offset
            self.target_z = self.home_position.z + self.target_altitude

    def control_loop(self) -> None:
        if self.have_pose:
            self.update_mission_target()

        self.pos_pub.publish(self.build_setpoint())

        if not self.state.connected or not self.have_pose:
            self.get_logger().info(
                "Waiting for FCU/pose...", throttle_duration_sec=5
            )
            return

        if self.phase == MissionPhase.PREFLIGHT:
            if self.preflight_counter < self.PREFLIGHT_SETPOINTS:
                self.preflight_counter += 1
                if self.preflight_counter == self.PREFLIGHT_SETPOINTS:
                    self.get_logger().info("Preflight setpoints sent, ready for OFFBOARD")
                return

            now = self.get_clock().now()
            if (now - self.last_req).nanoseconds / 1e9 < self.REQUEST_PERIOD:
                return

            if self.state.mode != "OFFBOARD":
                self.set_mode("OFFBOARD")
            elif not self.state.armed:
                self.arm()
            else:
                self.transition_to(MissionPhase.TAKEOFF)

            self.last_req = now
            return

        if self.phase in (MissionPhase.TAKEOFF, MissionPhase.TRANSIT, MissionPhase.HOLD):
            now = self.get_clock().now()
            if (
                self.state.mode != "OFFBOARD"
                and (now - self.last_req).nanoseconds / 1e9 >= self.REQUEST_PERIOD
            ):
                self.set_mode("OFFBOARD")
                self.last_req = now
                return

        if self.phase == MissionPhase.HOLD:
            self.get_logger().info(
                "Holding at target point",
                throttle_duration_sec=5,
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TakeoffPointHoldController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
