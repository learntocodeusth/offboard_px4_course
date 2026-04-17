#!/usr/bin/env python3

from __future__ import annotations

import math
from enum import Enum

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image


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
    SEARCH = "SEARCH"
    HOVER_DETECTED = "HOVER_DETECTED"


class ArucoTakeoffController(Node):
    TARGET_ALT = 2.5
    SETPOINT_PERIOD = 0.1
    REQUEST_PERIOD = 1.0
    PREFLIGHT_SETPOINTS = 20
    ASCENT_RATE = 0.8
    ALTITUDE_TOLERANCE = 0.20
    LAND_SETPOINT_ALT = 0.10
    SEARCH_X_MIN = 0.5
    SEARCH_X_MAX = 3.0
    SEARCH_Y_MIN = -1.0
    SEARCH_Y_MAX = 1.4
    SEARCH_LANE_SPACING = 0.8
    WAYPOINT_TOLERANCE = 0.25

    def __init__(self) -> None:
        super().__init__("aruco_takeoff_controller")

        self.declare_parameter("camera_topic", "/camera/down/image_raw")
        self.declare_parameter("aruco_id", 7)
        self.declare_parameter("target_altitude", self.TARGET_ALT)

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
        self.debug_pub = self.create_publisher(Image, "/aruco/debug_image", 10)

        self.arm_cli = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode_cli = self.create_client(SetMode, "/mavros/set_mode")

        self.create_subscription(State, "/mavros/state", self.state_cb, state_qos)
        self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self.pose_cb, pose_qos
        )
        self.create_subscription(
            Image,
            self.get_parameter("camera_topic").get_parameter_value().string_value,
            self.image_cb,
            qos_profile_sensor_data,
        )

        self.bridge = CvBridge()
        self.marker_id = (
            self.get_parameter("aruco_id").get_parameter_value().integer_value
        )
        self.target_altitude = (
            self.get_parameter("target_altitude").get_parameter_value().double_value
        )

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.aruco_detector = cv2.aruco.ArucoDetector(
                self.aruco_dict, cv2.aruco.DetectorParameters()
            )
        else:
            self.aruco_detector = None
            self.detector_params = cv2.aruco.DetectorParameters_create()

        self.state = State()
        self.pose = PoseStamped()
        self.have_pose = False
        self.have_image = False
        self.aruco_detected = False
        self.phase = MissionPhase.PREFLIGHT
        self.preflight_counter = 0
        self.last_req = self.get_clock().now()
        self.phase_started_at = self.get_clock().now()
        self.home_position = Point()
        self.target_xy = Point()
        self.hover_xy = Point()
        self.hover_z = self.LAND_SETPOINT_ALT
        self.target_z = self.LAND_SETPOINT_ALT
        self.target_orientation = euler_to_quaternion(0.0, 0.0, 0.0)
        self.search_waypoints: list[tuple[float, float]] = []
        self.current_waypoint_index = 0

        self.create_timer(self.SETPOINT_PERIOD, self.control_loop)
        self.get_logger().info(
            "Aruco takeoff controller initialised "
            f"(camera={self.get_parameter('camera_topic').value}, id={self.marker_id})"
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
        self.target_xy.x = msg.pose.position.x
        self.target_xy.y = msg.pose.position.y
        self.hover_xy.x = msg.pose.position.x
        self.hover_xy.y = msg.pose.position.y
        self.hover_z = msg.pose.position.z
        self.target_z = self.home_position.z + self.LAND_SETPOINT_ALT
        self.search_waypoints = self.build_search_waypoints()

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

    def image_cb(self, msg: Image) -> None:
        self.have_image = True

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # pragma: no cover - bridge errors depend on runtime
            self.get_logger().error(f"Image conversion failed: {exc}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.aruco_detector is not None:
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.detector_params
            )

        debug_frame = frame.copy()
        detected_ids: list[int] = []
        if ids is not None and len(ids) > 0:
            detected_ids = [int(marker_id) for marker_id in ids.flatten()]
            cv2.aruco.drawDetectedMarkers(debug_frame, corners, ids)

            if (
                self.marker_id in detected_ids
                and not self.aruco_detected
                and self.phase == MissionPhase.SEARCH
            ):
                self.aruco_detected = True
                self.hover_xy.x = self.pose.pose.position.x
                self.hover_xy.y = self.pose.pose.position.y
                self.hover_z = self.pose.pose.position.z
                self.target_xy.x = self.hover_xy.x
                self.target_xy.y = self.hover_xy.y
                self.target_z = self.hover_z
                self.transition_to(MissionPhase.HOVER_DETECTED)
                self.get_logger().info(f"Aruco detected (id={self.marker_id})")

        phase_text = f"phase={self.phase.value}"
        status_text = (
            f"aruco={'detected' if self.aruco_detected else 'searching'} "
            f"ids={detected_ids if detected_ids else '[]'}"
        )
        cv2.putText(
            debug_frame,
            phase_text,
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
        )
        cv2.putText(
            debug_frame,
            status_text,
            (20, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0) if self.aruco_detected else (0, 165, 255),
            2,
        )

        debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)

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
        if phase == MissionPhase.SEARCH:
            self.current_waypoint_index = 0
            self.set_search_target()
        elif phase == MissionPhase.HOVER_DETECTED:
            self.target_xy.x = self.hover_xy.x
            self.target_xy.y = self.hover_xy.y
            self.target_z = self.hover_z
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

    def build_search_waypoints(self) -> list[tuple[float, float]]:
        waypoints: list[tuple[float, float]] = []
        lane_index = 0
        y = self.SEARCH_Y_MIN

        while y <= self.SEARCH_Y_MAX + 1e-6:
            if lane_index % 2 == 0:
                xs = (self.SEARCH_X_MIN, self.SEARCH_X_MAX)
            else:
                xs = (self.SEARCH_X_MAX, self.SEARCH_X_MIN)

            for x in xs:
                waypoints.append(
                    (self.home_position.x + x, self.home_position.y + y)
                )

            lane_index += 1
            y += self.SEARCH_LANE_SPACING

        return waypoints

    def set_search_target(self) -> None:
        if not self.search_waypoints:
            return

        target_x, target_y = self.search_waypoints[self.current_waypoint_index]
        self.target_xy.x = target_x
        self.target_xy.y = target_y
        self.target_z = self.home_position.z + self.target_altitude

    def reached_xy_target(self) -> bool:
        dx = self.pose.pose.position.x - self.target_xy.x
        dy = self.pose.pose.position.y - self.target_xy.y
        return math.hypot(dx, dy) <= self.WAYPOINT_TOLERANCE

    def advance_search_waypoint(self) -> None:
        if not self.search_waypoints:
            return

        self.current_waypoint_index = (
            self.current_waypoint_index + 1
        ) % len(self.search_waypoints)
        self.set_search_target()
        self.get_logger().info(
            "Zigzag target -> "
            f"x={self.target_xy.x:.2f}, y={self.target_xy.y:.2f}",
        )

    def update_mission_target(self) -> None:
        if self.phase == MissionPhase.TAKEOFF:
            climb = self.ASCENT_RATE * self.seconds_in_phase()
            self.target_z = min(
                self.home_position.z + self.target_altitude,
                self.home_position.z + self.LAND_SETPOINT_ALT + climb,
            )
            if self.altitude_above_home() >= (
                self.target_altitude - self.ALTITUDE_TOLERANCE
            ):
                self.target_z = self.home_position.z + self.target_altitude
                self.transition_to(MissionPhase.SEARCH)
            return

        if self.phase == MissionPhase.SEARCH:
            self.target_z = self.home_position.z + self.target_altitude
            if self.reached_xy_target():
                self.advance_search_waypoint()
            return

        if self.phase == MissionPhase.HOVER_DETECTED:
            self.target_xy.x = self.hover_xy.x
            self.target_xy.y = self.hover_xy.y
            self.target_z = self.hover_z

    def control_loop(self) -> None:
        if self.have_pose:
            self.update_mission_target()

        self.pos_pub.publish(self.build_setpoint())

        if not self.state.connected or not self.have_pose:
            self.get_logger().info(
                "Waiting for FCU/pose...", throttle_duration_sec=5
            )
            return

        if not self.have_image:
            self.get_logger().info(
                "Waiting for downward camera images...", throttle_duration_sec=5
            )

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

        if self.phase in (MissionPhase.TAKEOFF, MissionPhase.SEARCH, MissionPhase.HOVER_DETECTED):
            now = self.get_clock().now()
            if (
                self.state.mode != "OFFBOARD"
                and (now - self.last_req).nanoseconds / 1e9 >= self.REQUEST_PERIOD
            ):
                self.set_mode("OFFBOARD")
                self.last_req = now
                return

        if self.phase == MissionPhase.HOVER_DETECTED:
            self.get_logger().info(
                "Holding position above detected ArUco marker",
                throttle_duration_sec=5,
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArucoTakeoffController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
