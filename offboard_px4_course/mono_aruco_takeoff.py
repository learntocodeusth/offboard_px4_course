#!/usr/bin/env python3

from __future__ import annotations

import math
import os
import shutil
import subprocess
import threading
from enum import Enum

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from gz.msgs10 import image_pb2 as gz_image_pb2
from gz.transport13 import Node as GzNode
from gz.transport13 import SubscribeOptions
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
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


def create_detector_parameters():
    if hasattr(cv2.aruco, "DetectorParameters"):
        return cv2.aruco.DetectorParameters()
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        return cv2.aruco.DetectorParameters_create()
    raise AttributeError("cv2.aruco does not provide DetectorParameters")


def create_aruco_detector(aruco_dict):
    detector_params = create_detector_parameters()
    if hasattr(detector_params, "cornerRefinementMethod") and hasattr(
        cv2.aruco, "CORNER_REFINE_SUBPIX"
    ):
        detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    if hasattr(cv2.aruco, "ArucoDetector"):
        return cv2.aruco.ArucoDetector(aruco_dict, detector_params), detector_params

    return None, detector_params


class MissionPhase(str, Enum):
    PREFLIGHT = "PREFLIGHT"
    TAKEOFF = "TAKEOFF"
    HOVER = "HOVER"


class MonoArucoTakeoffController(Node):
    DEFAULT_WORLD_NAME = os.environ.get("PX4_GZ_WORLD", "aruco_takeoff_world")
    DEFAULT_MODEL_NAME = os.environ.get("PX4_GZ_MODEL_NAME", "x500_mono_cam_down_0")
    DEFAULT_WORLD_CANDIDATES = ("aruco_takeoff_world", "default")
    DEFAULT_FALLBACK_TOPICS = ("/camera",)
    CAMERA_TOPIC_SUFFIX = "/link/camera_link/sensor/camera/image"
    TARGET_ALT = 2.0
    SETPOINT_PERIOD = 0.1
    PROCESS_PERIOD = 0.05
    REQUEST_PERIOD = 1.0
    PREFLIGHT_SETPOINTS = 20
    ASCENT_RATE = 0.8
    ALTITUDE_TOLERANCE = 0.20
    LAND_SETPOINT_ALT = 0.10

    def __init__(self) -> None:
        super().__init__("mono_aruco_takeoff_controller")

        self.declare_parameter("camera_topic", "")
        self.declare_parameter("world_name", self.DEFAULT_WORLD_NAME)
        self.declare_parameter("model_name", self.DEFAULT_MODEL_NAME)
        self.declare_parameter("camera_topic_autodiscover", True)
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
        self.debug_pub = self.create_publisher(Image, "/aruco/mono_takeoff_debug_image", 10)

        self.arm_cli = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode_cli = self.create_client(SetMode, "/mavros/set_mode")

        self.create_subscription(State, "/mavros/state", self.state_cb, state_qos)
        self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self.pose_cb, pose_qos
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
        self.world_name = (
            self.get_parameter("world_name").get_parameter_value().string_value.strip()
            or self.DEFAULT_WORLD_NAME
        )
        self.model_name = (
            self.get_parameter("model_name").get_parameter_value().string_value.strip()
            or self.DEFAULT_MODEL_NAME
        )

        self.camera_topics = self.resolve_camera_topics()
        self.camera_topic = self.camera_topics[0]
        self.sync_world_and_model_from_topic(self.camera_topic)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_detector, self.detector_params = create_aruco_detector(
            self.aruco_dict
        )

        self.gz_node = GzNode()
        self.gz_subscribed_topics: list[str] = []
        self.image_lock = threading.Lock()
        self.pending_bgr: np.ndarray | None = None
        self.pending_topic: str | None = None
        self.unsupported_pixel_formats: set[int] = set()
        self.active_camera_topic = ""

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

        self.subscribe_to_gz_camera_topics()

        self.create_timer(self.SETPOINT_PERIOD, self.control_loop)
        self.create_timer(self.PROCESS_PERIOD, self.process_pending_image)
        self.create_timer(1.0, self.camera_watchdog_cb)
        self.get_logger().info(
            "Mono ArUco takeoff controller initialised "
            f"(world={self.world_name}, model={self.model_name}, "
            f"camera={self.camera_topic}, candidates={len(self.camera_topics)}, "
            f"gz_subscriptions={len(self.gz_subscribed_topics)}, "
            f"aruco_id={self.marker_id}, target_altitude={self.target_altitude:.1f})"
        )

    def ordered_unique(self, values: list[str]) -> list[str]:
        seen = set()
        ordered: list[str] = []
        for value in values:
            if not value or value in seen:
                continue
            seen.add(value)
            ordered.append(value)
        return ordered

    def build_camera_topic(self, world_name: str, model_name: str) -> str:
        return f"/world/{world_name}/model/{model_name}{self.CAMERA_TOPIC_SUFFIX}"

    def sync_world_and_model_from_topic(self, camera_topic: str) -> None:
        parts = camera_topic.split("/")
        if len(parts) < 5 or parts[1] != "world" or parts[3] != "model":
            return
        self.world_name = parts[2]
        self.model_name = parts[4]

    def build_model_candidates(self) -> list[str]:
        configured_model = (
            self.get_parameter("model_name").get_parameter_value().string_value.strip()
        )
        env_model = os.environ.get("PX4_GZ_MODEL_NAME", "").strip()
        current_model = self.model_name
        family = current_model
        if "_" in family and family.rsplit("_", 1)[1].isdigit():
            family = family.rsplit("_", 1)[0]

        return self.ordered_unique(
            [
                configured_model,
                env_model,
                current_model,
                family,
                f"{family}_0",
                f"{family}_1",
                f"{family}_2",
                f"{family}_3",
                "x500_mono_cam_down",
                "x500_mono_cam_down_0",
            ]
        )

    def build_world_candidates(self) -> list[str]:
        configured_world = (
            self.get_parameter("world_name").get_parameter_value().string_value.strip()
        )
        env_world = os.environ.get("PX4_GZ_WORLD", "").strip()
        return self.ordered_unique(
            [
                configured_world,
                env_world,
                self.world_name,
                *self.DEFAULT_WORLD_CANDIDATES,
            ]
        )

    def query_gazebo_camera_topics(self) -> list[str]:
        if shutil.which("gz") is None:
            return []

        try:
            result = subprocess.run(
                ["gz", "topic", "-l"],
                check=False,
                capture_output=True,
                text=True,
                timeout=2.0,
            )
        except Exception as exc:  # pragma: no cover
            self.get_logger().debug(f"Failed to query Gazebo topics: {exc}")
            return []

        if result.returncode != 0:
            stderr = result.stderr.strip()
            if stderr:
                self.get_logger().debug(f"gz topic -l failed: {stderr}")
            return []

        topics = []
        for line in result.stdout.splitlines():
            topic = line.strip()
            if (
                topic.endswith(self.CAMERA_TOPIC_SUFFIX)
                or topic.endswith("/sensor/camera/image")
                or topic in self.DEFAULT_FALLBACK_TOPICS
            ):
                topics.append(topic)
        return topics

    def resolve_camera_topics(self) -> list[str]:
        configured_topic = (
            self.get_parameter("camera_topic").get_parameter_value().string_value.strip()
        )
        if configured_topic:
            return [configured_topic]

        preferred_topics = [
            self.build_camera_topic(world_name, model_name)
            for world_name in self.build_world_candidates()
            for model_name in self.build_model_candidates()
        ]
        preferred_topics = self.ordered_unique(
            [*preferred_topics, *self.DEFAULT_FALLBACK_TOPICS]
        )

        autodiscover = (
            self.get_parameter("camera_topic_autodiscover")
            .get_parameter_value()
            .bool_value
        )
        if not autodiscover:
            return preferred_topics

        available_topics = self.query_gazebo_camera_topics()
        if not available_topics:
            return preferred_topics

        model_candidates = self.build_model_candidates()
        prioritized_topics = [
            topic
            for topic in available_topics
            if topic in preferred_topics
            or any(f"/model/{model}/" in topic for model in model_candidates)
        ]
        fallback_topics = [
            topic for topic in available_topics if topic not in prioritized_topics
        ]
        return self.ordered_unique(
            [*prioritized_topics, *preferred_topics, *fallback_topics]
        )

    def subscribe_to_gz_camera_topics(self) -> None:
        options = SubscribeOptions()
        for topic in self.camera_topics:
            subscribed = self.gz_node.subscribe_raw(
                topic,
                lambda raw_msg, _info, topic=topic: self.gz_image_raw_cb(raw_msg, topic),
                "gz.msgs.Image",
                options,
            )
            if subscribed:
                self.gz_subscribed_topics.append(topic)
            else:
                self.get_logger().warn(f"Failed to subscribe to Gazebo image topic {topic}")

    def gz_image_raw_cb(self, raw_msg: bytes, topic: str) -> None:
        msg = gz_image_pb2.Image()
        try:
            msg.ParseFromString(raw_msg)
            bgr = self.convert_gz_image_to_bgr(msg)
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f"Failed to decode Gazebo image on {topic}: {exc}")
            return

        if bgr is None:
            return

        with self.image_lock:
            self.pending_bgr = bgr
            self.pending_topic = topic

    def convert_gz_image_to_bgr(self, msg: gz_image_pb2.Image) -> np.ndarray | None:
        if msg.width == 0 or msg.height == 0:
            return None

        pixel_format = msg.pixel_format_type
        if pixel_format == gz_image_pb2.RGB_INT8:
            channels = 3
        elif pixel_format == gz_image_pb2.BGR_INT8:
            channels = 3
        elif pixel_format == gz_image_pb2.RGBA_INT8:
            channels = 4
        elif pixel_format == gz_image_pb2.BGRA_INT8:
            channels = 4
        elif pixel_format == gz_image_pb2.L_INT8:
            channels = 1
        else:
            if pixel_format not in self.unsupported_pixel_formats:
                self.unsupported_pixel_formats.add(pixel_format)
                pixel_name = gz_image_pb2.PixelFormatType.Name(pixel_format)
                self.get_logger().warn(
                    f"Unsupported Gazebo pixel format {pixel_name}; only 8-bit RGB/BGR/RGBA/BGRA/L are handled."
                )
            return None

        row_stride = msg.step or (msg.width * channels)
        expected_size = msg.height * row_stride
        data = np.frombuffer(msg.data, dtype=np.uint8)
        if data.size < expected_size:
            self.get_logger().warn(
                "Received truncated Gazebo image payload.",
                throttle_duration_sec=5,
            )
            return None

        rows = data[:expected_size].reshape(msg.height, row_stride)
        pixel_bytes = msg.width * channels
        image = rows[:, :pixel_bytes]

        if channels == 1:
            gray = image.reshape(msg.height, msg.width)
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        image = image.reshape(msg.height, msg.width, channels)
        if pixel_format == gz_image_pb2.RGB_INT8:
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if pixel_format == gz_image_pb2.BGR_INT8:
            return image.copy()
        if pixel_format == gz_image_pb2.RGBA_INT8:
            return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        if pixel_format == gz_image_pb2.BGRA_INT8:
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        return None

    def process_pending_image(self) -> None:
        with self.image_lock:
            if self.pending_bgr is None or self.pending_topic is None:
                return
            bgr = self.pending_bgr
            topic = self.pending_topic
            self.pending_bgr = None
            self.pending_topic = None

        self.process_bgr_image(bgr, topic)

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

    def process_bgr_image(self, frame: np.ndarray, topic: str) -> None:
        self.have_image = True
        if topic != self.active_camera_topic:
            self.active_camera_topic = topic
            self.sync_world_and_model_from_topic(topic)
            self.get_logger().info(f"Using mono camera topic {topic}")

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

            marker_match = self.marker_id < 0 or self.marker_id in detected_ids
            if marker_match and not self.aruco_detected:
                matched_id = detected_ids[0] if self.marker_id < 0 else self.marker_id
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
                self.get_logger().info(
                    f"Aruco detected (id={matched_id}, topic={topic})"
                )

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

        if self.show_camera:
            cv2.imshow("Mono ArUco Takeoff", debug_frame)
            cv2.waitKey(1)

        debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")
        self.debug_pub.publish(debug_msg)

    def camera_watchdog_cb(self) -> None:
        if self.have_image:
            return
        self.get_logger().info(
            "Waiting for Gazebo mono camera images on "
            f"{self.camera_topic} (subscribed candidates={len(self.gz_subscribed_topics)})",
            throttle_duration_sec=5,
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
            self.get_logger().info("Waiting for FCU/pose...", throttle_duration_sec=5)
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
            if self.aruco_detected:
                message = "Holding position after ArUco detection"
            else:
                message = "Holding position above takeoff point while searching for ArUco"
            self.get_logger().info(message, throttle_duration_sec=5)

    def destroy_node(self) -> bool:
        for topic in self.gz_subscribed_topics:
            try:
                self.gz_node.unsubscribe(topic)
            except Exception:
                pass
        if self.show_camera:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MonoArucoTakeoffController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
