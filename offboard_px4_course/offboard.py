#!/usr/bin/env python3

from __future__ import annotations

import math

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


class OffboardController(Node):
    TARGET_ALT = 8.0
    REQUEST_PERIOD = 1.0
    SETPOINT_PERIOD = 0.1

    def __init__(self) -> None:
        super().__init__("offboard_controller")

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
        self.offboard = False
        self.last_req = self.get_clock().now()

        self.create_timer(self.SETPOINT_PERIOD, self.control_loop)
        self.get_logger().info("Offboard controller initialised")

    def state_cb(self, msg: State) -> None:
        self.state = msg

    def pose_cb(self, msg: PoseStamped) -> None:
        self.pose = msg
        if not self.have_pose and msg.header.stamp.sec:
            self.have_pose = True
            self.get_logger().info("Pose locked")

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

    def build_hold_setpoint(self) -> PoseStamped:
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = "map"

        if self.have_pose:
            sp.pose.position.x = self.pose.pose.position.x
            sp.pose.position.y = self.pose.pose.position.y
            sp.pose.position.z = self.TARGET_ALT
            sp.pose.orientation = self.pose.pose.orientation
        else:
            sp.pose.position = Point(x=0.0, y=0.0, z=0.1)
            sp.pose.orientation = euler_to_quaternion(0.0, 0.0, 0.0)

        return sp

    def control_loop(self) -> None:
        self.pos_pub.publish(self.build_hold_setpoint())

        if not self.state.connected or not self.have_pose:
            self.get_logger().info(
                "Waiting for FCU/pose...", throttle_duration_sec=5
            )
            return

        now = self.get_clock().now()
        if (now - self.last_req).nanoseconds / 1e9 < self.REQUEST_PERIOD:
            return

        if self.state.mode != "OFFBOARD":
            self.set_mode("OFFBOARD")
        elif not self.state.armed:
            self.arm()
        elif not self.offboard:
            self.offboard = True
            self.get_logger().info("OFFBOARD active, holding position")

        self.last_req = now


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
