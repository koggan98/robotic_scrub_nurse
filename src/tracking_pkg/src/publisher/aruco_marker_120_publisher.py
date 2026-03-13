#!/usr/bin/env python3

import time

import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener


TARGET_MARKER_ID = 120
TARGET_MARKER_SIZE_METERS = 0.045
TARGET_MARKER_FRAME = "aruco_marker_120_frame"
TOOL_HOLDER_FRAME = "tool_holder_frame"
TOOL_HOLDER_OFFSET_METERS = np.array([0.0, 0.040, -0.032], dtype=np.float64)
CAMERA_FRAME = "camera_frame"
CAMERA_FRAME_PARENT = "aruco_board_frame"
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)


def rotation_matrix_to_quaternion(rotation_matrix):
    """Convert a 3x3 rotation matrix into an xyzw quaternion."""
    trace = np.trace(rotation_matrix)

    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) * s
        y = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) * s
        z = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) * s
    else:
        diagonal = np.diag(rotation_matrix)
        max_index = int(np.argmax(diagonal))

        if max_index == 0:
            s = 2.0 * np.sqrt(1.0 + rotation_matrix[0, 0] - rotation_matrix[1, 1] - rotation_matrix[2, 2])
            w = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
            x = 0.25 * s
            y = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
            z = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
        elif max_index == 1:
            s = 2.0 * np.sqrt(1.0 + rotation_matrix[1, 1] - rotation_matrix[0, 0] - rotation_matrix[2, 2])
            w = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
            x = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
            y = 0.25 * s
            z = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1])
            w = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
            x = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
            y = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
            z = 0.25 * s

    quat = np.array([x, y, z, w], dtype=np.float64)
    norm = np.linalg.norm(quat)
    if norm == 0.0:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return quat / norm


class ArucoMarker120Publisher(Node):
    def __init__(self):
        super().__init__("aruco_marker_120_publisher")

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.marker_frame_published = False
        self.last_status_log_time = 0.0
        self.status_log_interval = 2.0

        self.detector_parameters = aruco.DetectorParameters()
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.marker_object_points = self._create_marker_object_points(TARGET_MARKER_SIZE_METERS)

        self.create_subscription(Image, "color_image", self.rgb_callback, 10)
        self.create_subscription(CameraInfo, "camera_info", self.camera_info_callback, 10)

        self.get_logger().info(
            f"Waiting for first valid detection of original ArUco marker {TARGET_MARKER_ID} "
            f"to publish static TFs {CAMERA_FRAME} -> {TARGET_MARKER_FRAME} "
            f"and {TARGET_MARKER_FRAME} -> {TOOL_HOLDER_FRAME}."
        )

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        # Use the camera-provided distortion coefficients when available.
        if len(msg.d) > 0:
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
        else:
            self.dist_coeffs = np.zeros((5,), dtype=np.float64)

    def rgb_callback(self, msg):
        if self.marker_frame_published:
            return

        if self.camera_matrix is None or self.dist_coeffs is None:
            self._log_status("Waiting for camera_info before ArUco marker 120 pose estimation can start.")
            return

        if not self._camera_frame_is_available():
            self._log_status(
                f"Waiting for TF {CAMERA_FRAME_PARENT} -> {CAMERA_FRAME} before locking "
                f"{TARGET_MARKER_FRAME}."
            )
            return

        color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(gray, ARUCO_DICT, parameters=self.detector_parameters)
        if ids is None or len(ids) == 0:
            self._log_status(
                f"Original ArUco marker {TARGET_MARKER_ID} not detected yet. "
                f"Static TF {CAMERA_FRAME} -> {TARGET_MARKER_FRAME} remains unlocked."
            )
            return

        ids_flat = ids.flatten()
        matching_indices = np.where(ids_flat == TARGET_MARKER_ID)[0]
        if len(matching_indices) == 0:
            self._log_status(
                f"Aruco markers detected, but marker {TARGET_MARKER_ID} is not among them. "
                f"Static TF {CAMERA_FRAME} -> {TARGET_MARKER_FRAME} remains unlocked."
            )
            return

        target_corners = corners[int(matching_indices[0])].reshape(4, 2)
        success, rvec, tvec = cv2.solvePnP(
            self.marker_object_points,
            target_corners,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )

        if not success:
            self._log_status(
                f"Marker {TARGET_MARKER_ID} was detected, but pose estimation failed. "
                f"Static TF {CAMERA_FRAME} -> {TARGET_MARKER_FRAME} remains unlocked."
            )
            return

        self.publish_locked_frames(rvec, tvec)

    def _camera_frame_is_available(self):
        return self.tf_buffer.can_transform(
            CAMERA_FRAME_PARENT,
            CAMERA_FRAME,
            rclpy.time.Time(),
            timeout=Duration(seconds=0.0),
        )

    def _create_marker_object_points(self, marker_size_meters):
        half_size = marker_size_meters / 2.0
        return np.array(
            [
                [-half_size, half_size, 0.0],
                [half_size, half_size, 0.0],
                [half_size, -half_size, 0.0],
                [-half_size, -half_size, 0.0],
            ],
            dtype=np.float32,
        )

    def _log_status(self, message):
        now = time.time()
        if now - self.last_status_log_time >= self.status_log_interval:
            self.get_logger().warn(message)
            self.last_status_log_time = now

    def publish_locked_frames(self, rvec, tvec):
        if self.marker_frame_published:
            return

        rotation_matrix, _ = cv2.Rodrigues(rvec)
        quat = rotation_matrix_to_quaternion(rotation_matrix)

        stamp = self.get_clock().now().to_msg()

        marker_transform = TransformStamped()
        marker_transform.header.stamp = stamp
        marker_transform.header.frame_id = CAMERA_FRAME
        marker_transform.child_frame_id = TARGET_MARKER_FRAME

        marker_transform.transform.translation.x = float(tvec[0][0])
        marker_transform.transform.translation.y = float(tvec[1][0])
        marker_transform.transform.translation.z = float(tvec[2][0])

        marker_transform.transform.rotation.x = float(quat[0])
        marker_transform.transform.rotation.y = float(quat[1])
        marker_transform.transform.rotation.z = float(quat[2])
        marker_transform.transform.rotation.w = float(quat[3])

        tool_holder_transform = TransformStamped()
        tool_holder_transform.header.stamp = stamp
        tool_holder_transform.header.frame_id = TARGET_MARKER_FRAME
        tool_holder_transform.child_frame_id = TOOL_HOLDER_FRAME

        tool_holder_transform.transform.translation.x = float(TOOL_HOLDER_OFFSET_METERS[0])
        tool_holder_transform.transform.translation.y = float(TOOL_HOLDER_OFFSET_METERS[1])
        tool_holder_transform.transform.translation.z = float(TOOL_HOLDER_OFFSET_METERS[2])

        tool_holder_transform.transform.rotation.x = 0.0
        tool_holder_transform.transform.rotation.y = 0.0
        tool_holder_transform.transform.rotation.z = 0.0
        tool_holder_transform.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform([marker_transform, tool_holder_transform])
        self.marker_frame_published = True
        self.get_logger().info(
            "Static TFs published after first valid marker detection: "
            f"{CAMERA_FRAME} -> {TARGET_MARKER_FRAME} and "
            f"{TARGET_MARKER_FRAME} -> {TOOL_HOLDER_FRAME}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ArucoMarker120Publisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
