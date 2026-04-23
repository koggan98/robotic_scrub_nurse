#!/usr/bin/env python3
"""
ArUco Marker Manager
====================
Single node that handles all ArUco marker-based camera localization and
static frame publishing, driven by config/aruco_markers.yaml.

Responsibilities:
  1. Load YAML config at startup.
  2. Publish static TFs for:
     - Each marker's known pose (parent_frame -> child_frame)
     - Each entry in static_frames (e.g. tool_holder_frame)
  3. For each marker with a detection config:
     - Subscribe to the camera's color/image_raw + color/camera_info
     - On first valid detection, publish a one-shot static TF
       (child_frame -> camera_output_frame), i.e. the camera's pose in
       the marker frame.

Config: config/aruco_markers.yaml
"""

import os
import time

import cv2
import cv2.aruco as aruco
import numpy as np
import yaml

import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster


ARUCO_DICT_MAP = {
    "original": aruco.DICT_ARUCO_ORIGINAL,
    "4x4_50": aruco.DICT_4X4_50,
    "4x4_100": aruco.DICT_4X4_100,
    "4x4_250": aruco.DICT_4X4_250,
    "5x5_100": aruco.DICT_5X5_100,
    "5x5_250": aruco.DICT_5X5_250,
    "6x6_250": aruco.DICT_6X6_250,
}


def _rpy_to_quat(rpy):
    """Convert [roll, pitch, yaw] (radians) to xyzw quaternion (ROS convention)."""
    return R.from_euler('xyz', rpy).as_quat()


def _make_static_tf(stamp, parent_frame, child_frame, translation, quat_xyzw):
    """Build a TransformStamped message."""
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame
    t.transform.translation.x = float(translation[0])
    t.transform.translation.y = float(translation[1])
    t.transform.translation.z = float(translation[2])
    t.transform.rotation.x = float(quat_xyzw[0])
    t.transform.rotation.y = float(quat_xyzw[1])
    t.transform.rotation.z = float(quat_xyzw[2])
    t.transform.rotation.w = float(quat_xyzw[3])
    return t


def _invert_pose(rvec, tvec):
    """
    Invert a pose given as (rvec, tvec).

    Input:  pose of marker expressed in camera frame
            (solvePnP output: marker_in_camera)
    Output: pose of camera expressed in marker frame
            (quat_xyzw, translation_xyz)
    """
    rot, _ = cv2.Rodrigues(rvec)
    rot_inv = rot.T
    t_inv = -rot_inv @ tvec.reshape(3)
    quat = R.from_matrix(rot_inv).as_quat()
    return quat, t_inv


class _MarkerTracker:
    """Per-marker detection state. Subscribes to one camera, locks on first detection."""

    def __init__(self, node, marker_cfg, detector_params, tf_broadcaster):
        self.node = node
        self.cfg = marker_cfg
        self.detector_params = detector_params
        self.tf_broadcaster = tf_broadcaster

        self.marker_id = int(marker_cfg['id'])
        self.marker_size = float(marker_cfg['size'])
        self.static_pose = marker_cfg['static_pose']
        self.detection = marker_cfg['detection']

        self.parent_frame = self.static_pose['parent_frame']
        self.child_frame = self.static_pose['child_frame']
        self.camera_ns = self.detection['camera_namespace'].rstrip('/')
        self.camera_output_frame = self.detection['camera_output_frame']
        self.publish_once = bool(self.detection.get('publish_once', True))

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.detection_locked = False

        self.last_log_time = 0.0
        self.log_interval = 2.0

        self.object_points = self._make_object_points(self.marker_size)

        # Build dictionary from parent config (same for all markers)
        self.aruco_dict = node.aruco_dictionary

        # Subscribe to this camera's color + camera_info
        color_topic = f"{self.camera_ns}/color/image_raw"
        info_topic = f"{self.camera_ns}/color/camera_info"

        node.create_subscription(
            CameraInfo, info_topic, self._on_camera_info, 10
        )
        node.create_subscription(
            Image, color_topic, self._on_image, 10
        )

        node.get_logger().info(
            f"[{marker_cfg['name']}] Watching marker id={self.marker_id} "
            f"(size={self.marker_size} m) on {color_topic}; "
            f"will publish TF {self.child_frame} -> {self.camera_output_frame} "
            f"on first detection"
        )

    @staticmethod
    def _make_object_points(size_m):
        h = size_m / 2.0
        return np.array(
            [[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]],
            dtype=np.float32,
        )

    def _on_camera_info(self, msg):
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        if len(msg.d) > 0:
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
        else:
            self.dist_coeffs = np.zeros((5,), dtype=np.float64)

    def _log_status(self, message):
        now = time.time()
        if now - self.last_log_time >= self.log_interval:
            self.node.get_logger().warn(f"[{self.cfg['name']}] {message}")
            self.last_log_time = now

    def _on_image(self, msg):
        if self.detection_locked and self.publish_once:
            return
        if self.camera_matrix is None or self.dist_coeffs is None:
            self._log_status("Waiting for camera_info before pose estimation can start.")
            return

        color = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.detector_params
        )
        if ids is None or len(ids) == 0:
            self._log_status(
                f"Marker {self.marker_id} not detected yet. "
                f"TF {self.child_frame} -> {self.camera_output_frame} remains unlocked."
            )
            return

        matches = np.where(ids.flatten() == self.marker_id)[0]
        if len(matches) == 0:
            self._log_status(
                f"Detected markers do not include id {self.marker_id}. "
                f"TF {self.child_frame} -> {self.camera_output_frame} remains unlocked."
            )
            return

        target_corners = corners[int(matches[0])].reshape(4, 2)
        ok, rvec, tvec = cv2.solvePnP(
            self.object_points,
            target_corners,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )

        if not ok:
            self._log_status(
                f"Marker {self.marker_id} detected, but pose estimation failed."
            )
            return

        self._publish_camera_pose(rvec, tvec)

    def _publish_camera_pose(self, rvec, tvec):
        # solvePnP gives marker_in_camera. We want camera_in_marker.
        quat_inv, t_inv = _invert_pose(rvec, tvec)

        stamp = self.node.get_clock().now().to_msg()
        tf_msg = _make_static_tf(
            stamp,
            parent_frame=self.child_frame,
            child_frame=self.camera_output_frame,
            translation=t_inv,
            quat_xyzw=quat_inv,
        )
        self.tf_broadcaster.sendTransform([tf_msg])
        self.detection_locked = True
        self.node.get_logger().info(
            f"[{self.cfg['name']}] Locked: published static TF "
            f"{self.child_frame} -> {self.camera_output_frame}"
        )


class ArucoMarkerManager(Node):
    def __init__(self):
        super().__init__('aruco_marker_manager')

        self.declare_parameter('config_path', '')
        cfg_path = self.get_parameter('config_path').value
        if not cfg_path:
            pkg_share = get_package_share_directory('tracking_pkg')
            cfg_path = os.path.join(pkg_share, 'config', 'aruco_markers.yaml')

        self.config = self._load_config(cfg_path)

        dict_name = self.config.get('dictionary', 'original')
        if dict_name not in ARUCO_DICT_MAP:
            self.get_logger().error(
                f"Unknown ArUco dictionary: {dict_name}. Falling back to 'original'."
            )
            dict_name = 'original'
        self.aruco_dictionary = aruco.getPredefinedDictionary(ARUCO_DICT_MAP[dict_name])

        self.detector_parameters = aruco.DetectorParameters()
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        self._publish_static_tfs()
        self._spawn_marker_trackers()

        self.get_logger().info(
            f"ArucoMarkerManager initialized ({len(self.config.get('markers', []))} markers, "
            f"{len(self.config.get('static_frames', []))} static frames, dict={dict_name})"
        )

    def _load_config(self, path):
        try:
            with open(path, 'r') as f:
                cfg = yaml.safe_load(f)
            self.get_logger().info(f"Loaded ArUco config from {path}")
            return cfg
        except Exception as e:
            self.get_logger().error(f"Failed to load ArUco config from {path}: {e}")
            return {}

    def _publish_static_tfs(self):
        """Publish all static TFs: per-marker static_pose + static_frames entries."""
        transforms = []
        stamp = self.get_clock().now().to_msg()

        for marker in self.config.get('markers', []):
            sp = marker.get('static_pose', {})
            if not sp:
                continue
            quat = _rpy_to_quat(sp.get('rotation_rpy', [0.0, 0.0, 0.0]))
            transforms.append(_make_static_tf(
                stamp,
                parent_frame=sp['parent_frame'],
                child_frame=sp['child_frame'],
                translation=sp.get('translation', [0.0, 0.0, 0.0]),
                quat_xyzw=quat,
            ))

        for sf in self.config.get('static_frames', []):
            quat = _rpy_to_quat(sf.get('rotation_rpy', [0.0, 0.0, 0.0]))
            transforms.append(_make_static_tf(
                stamp,
                parent_frame=sf['parent_frame'],
                child_frame=sf['child_frame'],
                translation=sf.get('translation', [0.0, 0.0, 0.0]),
                quat_xyzw=quat,
            ))

        if transforms:
            self.static_tf_broadcaster.sendTransform(transforms)
            self.get_logger().info(f"Published {len(transforms)} static TFs from config")

    def _spawn_marker_trackers(self):
        """Create one _MarkerTracker per marker entry with a detection config."""
        self.trackers = []
        for marker in self.config.get('markers', []):
            if 'detection' not in marker:
                continue
            tracker = _MarkerTracker(
                self, marker, self.detector_parameters, self.static_tf_broadcaster
            )
            self.trackers.append(tracker)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoMarkerManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
