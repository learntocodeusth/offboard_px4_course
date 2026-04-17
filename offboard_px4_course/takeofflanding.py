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
    HOLD = "HOLD"
    LAND = "LAND"
    COMPLETE = "COMPLETE"


class OffboardController(Node):
    TARGET_ALT = 8.0
    HOLD_DURATION = 5.0
    SETPOINT_PERIOD = 0.1
    REQUEST_PERIOD = 1.0
    LAND_MODE = "AUTO.LAND"
    PREFLIGHT_SETPOINTS = 20
    ASCENT_RATE = 1.0
    DESCENT_RATE = 0.7
    ALTITUDE_TOLERANCE = 0.25
    LAND_SETPOINT_ALT = 0.10
    DISARM_ALT = 0.15

    def __init__(self) -> None:
        super().__init__("offboard_takeoff_landing_controller")

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

        self.state = State()
        self.pose = PoseStamped()
        self.have_pose = False
        self.phase = MissionPhase.PREFLIGHT
        self.preflight_counter = 0
        self.last_req = self.get_clock().now()
        self.phase_started_at = self.get_clock().now()
        self.land_mode_requested = False
        self.home_position = Point()
        self.target_xy = Point()
        self.target_z = self.LAND_SETPOINT_ALT
        self.target_orientation = euler_to_quaternion(0.0, 0.0, 0.0)

        self.create_timer(self.SETPOINT_PERIOD, self.control_loop)
        self.get_logger().info("Offboard takeoff/landing controller initialised")

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
        self.target_xy.x = msg.pose.position.x
        self.target_xy.y = msg.pose.position.y
        self.target_z = self.home_position.z + self.LAND_SETPOINT_ALT

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

    def disarm(self) -> None:
        self._call(self.arm_cli, CommandBool.Request(value=False), "Disarm")

    def set_mode(self, mode: str) -> None:
        self._call(self.mode_cli, SetMode.Request(custom_mode=mode), f"Mode {mode}")

    def transition_to(self, phase: MissionPhase) -> None:
        self.phase = phase
        self.phase_started_at = self.get_clock().now()
        if phase == MissionPhase.LAND:
            self.land_mode_requested = False
            self.target_z = self.pose.pose.position.z
        self.get_logger().info(f"Mission phase -> {phase.value}")

    def seconds_in_phase(self) -> float:
        now = self.get_clock().now()
        return (now - self.phase_started_at).nanoseconds / 1e9

    def altitude_above_home(self) -> float:
        return self.pose.pose.position.z - self.home_position.z

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
            climb = self.ASCENT_RATE * self.seconds_in_phase()
            self.target_z = min(
                self.home_position.z + self.TARGET_ALT,
                self.home_position.z + self.LAND_SETPOINT_ALT + climb,
            )
            if self.altitude_above_home() >= (
                self.TARGET_ALT - self.ALTITUDE_TOLERANCE
            ):
                self.target_z = self.home_position.z + self.TARGET_ALT
                self.transition_to(MissionPhase.HOLD)
            return

        if self.phase == MissionPhase.HOLD:
            self.target_z = self.home_position.z + self.TARGET_ALT
            if self.seconds_in_phase() >= self.HOLD_DURATION:
                self.transition_to(MissionPhase.LAND)
            return

        if self.phase == MissionPhase.LAND:
            if not self.state.armed:
                self.transition_to(MissionPhase.COMPLETE)
            return

        if self.phase == MissionPhase.COMPLETE:
            self.target_z = self.home_position.z + self.LAND_SETPOINT_ALT

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

        if self.phase in (MissionPhase.TAKEOFF, MissionPhase.HOLD):
            now = self.get_clock().now()
            if (
                self.state.mode != "OFFBOARD"
                and (now - self.last_req).nanoseconds / 1e9 >= self.REQUEST_PERIOD
            ):
                self.set_mode("OFFBOARD")
                self.last_req = now
                return

        if self.phase == MissionPhase.LAND and self.state.armed:
            now = self.get_clock().now()
            if (
                self.state.mode != self.LAND_MODE
                and (now - self.last_req).nanoseconds / 1e9 >= self.REQUEST_PERIOD
            ):
                self.set_mode(self.LAND_MODE)
                self.land_mode_requested = True
                self.last_req = now
            elif self.state.mode == self.LAND_MODE and self.land_mode_requested:
                self.get_logger().info(
                    "AUTO.LAND active, waiting for PX4 to land and disarm",
                    throttle_duration_sec=5,
                )
            return

        if self.phase == MissionPhase.COMPLETE:
            self.get_logger().info("Mission complete", throttle_duration_sec=5)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OffboardController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
