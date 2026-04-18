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
    HOVER = "HOVER"


class GimbalArucoTakeoffController(Node):
    IMAGE_TOPIC = "/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image"
    IMG_W = 1280
    IMG_H = 720
    HFOV_RAD = 2.0
    MARKER_SIZE_M = 0.2
    MARKER_TIMEOUT_S = 0.5
    TARGET_ALT = 2.0
    SETPOINT_PERIOD = 0.1
    REQUEST_PERIOD = 1.0
    PREFLIGHT_SETPOINTS = 20
    ASCENT_RATE = 0.8
    ALTITUDE_TOLERANCE = 0.20
    LAND_SETPOINT_ALT = 0.10

    def __init__(self) -> None:
        super().__init__("gimbal_aruco_takeoff_controller")

        self.declare_parameter("camera_topic", self.IMAGE_TOPIC)
        self.declare_parameter("aruco_id", -1)
        self.declare_parameter("target_altitude", self.TARGET_ALT)
        self.declare_parameter("show_camera", True)

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
        self.debug_pub = self.create_publisher(Image, "/aruco/gimbal_debug_image", 10)

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
        self.show_camera = (
            self.get_parameter("show_camera").get_parameter_value().bool_value
        )

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        detector_params = cv2.aruco.DetectorParameters()
        detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.aruco_detector = cv2.aruco.ArucoDetector(
                self.aruco_dict, detector_params
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
        self.last_marker_seen = self.get_clock().now()

        self.create_timer(self.SETPOINT_PERIOD, self.control_loop)
        self.get_logger().info(
            "Gimbal ArUco takeoff controller initialised "
            f"(camera={self.get_parameter('camera_topic').value}, "
            f"img={self.IMG_W}x{self.IMG_H}, hfov={self.HFOV_RAD:.2f} rad, "
            f"marker_size={self.MARKER_SIZE_M:.2f} m)"
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
        if msg.encoding in ("16UC1", "32FC1"):
            self.get_logger().warn(
                "Received depth image. Gimbal ArUco detection expects RGB images.",
                throttle_duration_sec=5,
            )
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f"Image conversion failed: {exc}")
            return

        if bgr.shape[1] != self.IMG_W or bgr.shape[0] != self.IMG_H:
            bgr = cv2.resize(bgr, (self.IMG_W, self.IMG_H))

        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        if self.aruco_detector is not None:
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.detector_params
            )

        detected_ids: list[int] = []
        debug = bgr.copy()
        if ids is not None and len(ids) > 0:
            detected_ids = [int(marker_id) for marker_id in ids.flatten()]
            cv2.aruco.drawDetectedMarkers(debug, corners, ids)
            self.last_marker_seen = self.get_clock().now()

            marker_match = self.marker_id < 0 or self.marker_id in detected_ids
            if marker_match:
                matched_id = detected_ids[0] if self.marker_id < 0 else self.marker_id
                if not self.aruco_detected:
                    self.get_logger().info(f"Aruco detected (id={matched_id})")
                self.aruco_detected = True
                self.hover_xy.x = self.pose.pose.position.x
                self.hover_xy.y = self.pose.pose.position.y
                self.hover_z = max(
                    self.pose.pose.position.z,
                    self.home_position.z + self.target_altitude,
                )
                self.target_xy.x = self.hover_xy.x
                self.target_xy.y = self.hover_xy.y
                self.target_z = self.hover_z

        marker_age = (self.get_clock().now() - self.last_marker_seen).nanoseconds / 1e9
        if marker_age > self.MARKER_TIMEOUT_S:
            self.aruco_detected = False

        status = "detected" if self.aruco_detected else "searching"
        cv2.putText(
            debug,
            f"phase={self.phase.value}",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
        )
        cv2.putText(
            debug,
            f"aruco={status} ids={detected_ids if detected_ids else '[]'}",
            (20, 65),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0) if self.aruco_detected else (0, 165, 255),
            2,
        )
        cv2.putText(
            debug,
            f"marker_timeout={self.MARKER_TIMEOUT_S:.1f}s",
            (20, 100),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 0),
            2,
        )

        if self.show_camera:
            cv2.imshow("Gimbal Camera", bgr)
            cv2.waitKey(1)

        debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
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

    def set_mode(self, mode: str) -> None:
        self._call(self.mode_cli, SetMode.Request(custom_mode=mode), f"Mode {mode}")

    def transition_to(self, phase: MissionPhase) -> None:
        self.phase = phase
        self.phase_started_at = self.get_clock().now()
        if phase == MissionPhase.HOVER:
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

    def update_mission_target(self) -> None:
        if self.phase == MissionPhase.TAKEOFF:
            self.target_xy.x = self.home_position.x
            self.target_xy.y = self.home_position.y
            climb = self.ASCENT_RATE * self.seconds_in_phase()
            self.target_z = min(
                self.home_position.z + self.target_altitude,
                self.home_position.z + self.LAND_SETPOINT_ALT + climb,
            )
            if self.altitude_above_home() >= (
                self.target_altitude - self.ALTITUDE_TOLERANCE
            ):
                self.hover_xy.x = self.pose.pose.position.x
                self.hover_xy.y = self.pose.pose.position.y
                self.hover_z = self.home_position.z + self.target_altitude
                self.transition_to(MissionPhase.HOVER)
            return

        if self.phase == MissionPhase.HOVER:
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
                "Waiting for gimbal camera images...", throttle_duration_sec=5
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

        if self.phase in (MissionPhase.TAKEOFF, MissionPhase.HOVER):
            now = self.get_clock().now()
            if (
                self.state.mode != "OFFBOARD"
                and (now - self.last_req).nanoseconds / 1e9 >= self.REQUEST_PERIOD
            ):
                self.set_mode("OFFBOARD")
                self.last_req = now
                return

        if self.phase == MissionPhase.HOVER:
            self.get_logger().info(
                "Holding position with gimbal camera pointed down",
                throttle_duration_sec=5,
            )

    def destroy_node(self) -> bool:
        if self.show_camera:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GimbalArucoTakeoffController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
