#!/usr/bin/env python3
"""
World Model Builder
===================
On-demand service node that captures one fresh frame from the tray camera,
runs YOLOv8-OBB inference, pairs handle/body OBBs per tool, computes a grasp
point and orientation for each tool, projects everything to world frame, and
returns the result as both a structured JSON string (for the LLM) and ROS
messages (for downstream ROS subscribers).

Service:
  /build_world_model (tracking_pkg/srv/BuildWorldModel)

Continuous publishers (updated only on service call, not streamed):
  /world_model/json (std_msgs/String)            - JSON snapshot
  /world_model/tools_detected (ToolDetectionArray)
  /world_model/grasp_candidates (GraspCandidateArray)
  /world_model/annotated_image (sensor_msgs/Image) - debug overlay

Parameters:
  model_path (str): YOLOv8-OBB weights, e.g. .../first_obb_test.pt
  knowledge_base_path (str): tool_knowledge_base.yaml (defaults to package config)
  tray_camera_namespace (str): default "/tray_camera"
  tray_camera_frame (str):     default "tray_camera_color_optical_frame"
  world_frame (str):           default "world"
  conf_threshold (float):      default 0.35
  imgsz (int):                 inference size, default 1024
  device (str):                "cpu" or "cuda:0", default "cpu"
  handle_class_name (str):     OBB class that represents handles, default "handle"
  grasp_offset_fraction (float): grasp point offset along long axis from handle
                                 center toward body center, in fractions of image
                                 height. Default 1/16 = 0.0625.
  fixed_tool_plane_z_m (float):  optional world-frame z plane for grasp pixel
                                 projection. If finite, grasp/world directions
                                 use ray-plane projection instead of depth.
  depth_search_radius_px (int):  radius for median depth fallback around a
                                 target pixel. Default 5.
  depth_min_m / depth_max_m:     valid depth range used by the median fallback.
  max_image_age_sec (float):   reject service if image older than this, default 1.0

Grasp geometry:
  - Long axis of body OBB = the longer side direction.
  - Sign of (handle_center - body_center) projected on long axis tells which
    "side" the handle is on; grasp point is on the opposite side along the
    axis from the handle.
  - grasp_point_px = handle_center - sign * (grasp_offset_fraction * image_h) * long_axis_unit
  - 3D = depth-projection at that pixel + TF lookup.
"""

import json
import math
import os
import time
from threading import Lock

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import (
    Point,
    Point32,
    PoseStamped,
    Vector3,
)
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
import tf2_ros

from tracking_pkg.msg import (
    GraspCandidate,
    GraspCandidateArray,
    OrientedBoundingBox2D,
    ToolDetection,
    ToolDetectionArray,
)
from tracking_pkg.srv import BuildWorldModel


# ─── Geometry helpers ────────────────────────────────────────────────


def _obb_long_axis(width, height, angle_rad):
    """Return (axis_angle_rad, axis_length_px) for an OBB's longer side."""
    if width >= height:
        return angle_rad, float(width)
    return angle_rad + math.pi / 2.0, float(height)


def _project_onto_axis(p_from, p_to, axis_unit):
    """Signed scalar projection of (p_to - p_from) onto axis_unit."""
    delta = np.asarray(p_to, dtype=float) - np.asarray(p_from, dtype=float)
    return float(np.dot(delta, axis_unit))


def _point_in_obb(pt, corners):
    """True if pt (x,y) lies inside the OBB defined by 4 corner points."""
    poly = np.asarray(corners, dtype=np.float32).reshape(-1, 2)
    return cv2.pointPolygonTest(poly, (float(pt[0]), float(pt[1])), False) >= 0.0


def _tf_to_matrix(tf_msg):
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    rot = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = [t.x, t.y, t.z]
    return T


# ─── Detection container ─────────────────────────────────────────────


class _Detection:
    """Single OBB detection from the model."""

    def __init__(self, cls_name, conf, center, w, h, angle_rad, corners):
        self.cls_name = cls_name
        self.conf = float(conf)
        self.center = np.array(center, dtype=float)  # (cx, cy)
        self.width = float(w)
        self.height = float(h)
        self.angle_rad = float(angle_rad)
        self.corners = np.array(corners, dtype=float).reshape(4, 2)

    def to_msg(self):
        msg = OrientedBoundingBox2D()
        msg.center_x = float(self.center[0])
        msg.center_y = float(self.center[1])
        msg.width = float(self.width)
        msg.height = float(self.height)
        msg.angle_rad = float(self.angle_rad)
        msg.corners = [Point32(x=float(c[0]), y=float(c[1]), z=0.0)
                       for c in self.corners]
        return msg

    def to_dict(self):
        return {
            "center_xy": self.center.tolist(),
            "width_px": self.width,
            "height_px": self.height,
            "angle_rad": self.angle_rad,
            "corners": self.corners.tolist(),
        }


# ─── Builder Node ────────────────────────────────────────────────────


class WorldModelBuilder(Node):
    def __init__(self):
        super().__init__('world_model_builder')

        # ── Parameters ─────────────────────────────────────────
        self.declare_parameter('model_path', '')
        self.declare_parameter('knowledge_base_path', '')
        # Two camera-access modes:
        #   - "direct"    : node opens pyrealsense2 pipeline itself (low CPU, no
        #                   continuous ROS image publishing - recommended for
        #                   compute-constrained hosts like the NUC)
        #   - "streaming" : subscribe to a realsense2_camera topic stream
        self.declare_parameter('camera_mode', 'direct')
        self.declare_parameter('realsense_serial', '')
        self.declare_parameter('color_width', 1280)
        self.declare_parameter('color_height', 720)
        self.declare_parameter('color_fps', 30)
        self.declare_parameter('warmup_frames', 5)
        # Streaming-mode topic prefix (only used when camera_mode == "streaming")
        self.declare_parameter('tray_camera_namespace', '/tray_camera')
        self.declare_parameter('tray_camera_frame', 'tray_camera_color_optical_frame')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('conf_threshold', 0.35)
        self.declare_parameter('imgsz', 1024)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('handle_class_name', 'handle')
        self.declare_parameter('grasp_offset_fraction', 1.0 / 16.0)
        self.declare_parameter('fixed_tool_plane_z_m', float('nan'))
        self.declare_parameter('depth_search_radius_px', 5)
        self.declare_parameter('depth_min_m', 0.05)
        self.declare_parameter('depth_max_m', 2.0)
        self.declare_parameter('max_image_age_sec', 1.0)

        self.model_path = self.get_parameter('model_path').value
        kb_path = self.get_parameter('knowledge_base_path').value
        if not kb_path:
            kb_path = os.path.join(
                get_package_share_directory('tracking_pkg'),
                'config', 'tool_knowledge_base.yaml',
            )
        self.knowledge_base = self._load_knowledge_base(kb_path)

        self.camera_mode = self.get_parameter('camera_mode').value
        self.realsense_serial = self.get_parameter('realsense_serial').value
        self.color_width = int(self.get_parameter('color_width').value)
        self.color_height = int(self.get_parameter('color_height').value)
        self.color_fps = int(self.get_parameter('color_fps').value)
        self.warmup_frames = int(self.get_parameter('warmup_frames').value)
        ns = self.get_parameter('tray_camera_namespace').value.rstrip('/')
        self.tray_camera_frame = self.get_parameter('tray_camera_frame').value
        self.world_frame = self.get_parameter('world_frame').value
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)
        self.imgsz = int(self.get_parameter('imgsz').value)
        self.device = self.get_parameter('device').value
        self.handle_class_name = self.get_parameter('handle_class_name').value
        self.grasp_offset_fraction = float(self.get_parameter('grasp_offset_fraction').value)
        self.fixed_tool_plane_z_m = float(self.get_parameter('fixed_tool_plane_z_m').value)
        self.depth_search_radius_px = int(self.get_parameter('depth_search_radius_px').value)
        self.depth_min_m = float(self.get_parameter('depth_min_m').value)
        self.depth_max_m = float(self.get_parameter('depth_max_m').value)
        self.max_image_age_sec = float(self.get_parameter('max_image_age_sec').value)

        # ── State ──────────────────────────────────────────────
        self.bridge = CvBridge()
        self._lock = Lock()
        self._latest_image = None
        self._latest_image_stamp = None
        self._latest_depth = None
        self._latest_depth_stamp = None
        self._camera_matrix = None
        self._dist_coeffs = None

        # Direct (pyrealsense2) pipeline state
        self._rs = None
        self._rs_pipeline = None
        self._rs_align = None
        self._direct_camera_matrix = None
        self._direct_lock = Lock()

        self._model = None  # lazy load
        self._tool_seq = 0

        # ── TF ─────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Camera input setup ────────────────────────────────
        if self.camera_mode == 'streaming':
            self.create_subscription(
                Image, f'{ns}/color/image_raw', self._on_color, 10
            )
            self.create_subscription(
                Image, f'{ns}/aligned_depth_to_color/image_raw', self._on_depth, 10
            )
            self.create_subscription(
                CameraInfo, f'{ns}/color/camera_info', self._on_camera_info, 10
            )
            self.get_logger().info(
                f'Camera mode: streaming (subscribing to {ns}/color/image_raw)'
            )
        elif self.camera_mode == 'direct':
            try:
                self._open_direct_pipeline()
            except Exception as e:
                self.get_logger().error(f'Failed to open direct RealSense pipeline: {e}')
        else:
            self.get_logger().error(
                f'Unknown camera_mode "{self.camera_mode}" - use "direct" or "streaming"'
            )

        # ── Publishers (updated only on service call) ─────────
        self.json_pub = self.create_publisher(String, '/world_model/json', 10)
        self.tools_pub = self.create_publisher(
            ToolDetectionArray, '/world_model/tools_detected', 10
        )
        self.grasp_pub = self.create_publisher(
            GraspCandidateArray, '/world_model/grasp_candidates', 10
        )
        self.annotated_pub = self.create_publisher(
            Image, '/world_model/annotated_image', 10
        )

        # ── Service ────────────────────────────────────────────
        self.srv = self.create_service(
            BuildWorldModel, '/build_world_model', self._on_service
        )

        self.get_logger().info(
            f'WorldModelBuilder ready (mode={self.camera_mode}, '
            f'model={self.model_path or "<unset>"}, imgsz={self.imgsz}, device={self.device})'
        )

    # ── Direct pyrealsense2 pipeline ───────────────────────────────

    def _open_direct_pipeline(self):
        """Open a pyrealsense2 pipeline once at startup. Frames are pulled
        from this pipeline only when /build_world_model is called."""
        import pyrealsense2 as rs
        self._rs = rs

        pipeline = rs.pipeline()
        config = rs.config()
        if self.realsense_serial:
            config.enable_device(self.realsense_serial)
        config.enable_stream(
            rs.stream.color,
            self.color_width, self.color_height,
            rs.format.bgr8, self.color_fps,
        )
        config.enable_stream(
            rs.stream.depth,
            self.color_width, self.color_height,
            rs.format.z16, self.color_fps,
        )

        profile = pipeline.start(config)
        self._rs_pipeline = pipeline
        self._rs_align = rs.align(rs.stream.color)

        # Read intrinsics from the actually negotiated color stream
        color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = color_profile.get_intrinsics()
        self._direct_camera_matrix = np.array(
            [
                [intr.fx, 0.0, intr.ppx],
                [0.0, intr.fy, intr.ppy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

        # Warm up - the first few frames have unsettled exposure/depth
        for _ in range(max(0, self.warmup_frames)):
            try:
                pipeline.wait_for_frames(timeout_ms=2000)
            except Exception:
                break

        self.get_logger().info(
            f'Direct RealSense pipeline open: {self.color_width}x{self.color_height}@'
            f'{self.color_fps} fps, serial={self.realsense_serial or "<first available>"}, '
            f'fx={intr.fx:.1f} fy={intr.fy:.1f} cx={intr.ppx:.1f} cy={intr.ppy:.1f}'
        )

    def _capture_direct(self):
        """Pull one fresh aligned color+depth frame from the open pipeline."""
        if self._rs_pipeline is None:
            return None, None, None

        with self._direct_lock:
            # Drain the queue to make sure we hand back a fresh frame
            while True:
                try:
                    polled = self._rs_pipeline.poll_for_frames()
                except Exception:
                    polled = None
                if not polled:
                    break

            try:
                frames = self._rs_pipeline.wait_for_frames(timeout_ms=2000)
            except Exception as e:
                self.get_logger().error(f'wait_for_frames failed: {e}')
                return None, None, None

            aligned = self._rs_align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if not color_frame or not depth_frame:
                return None, None, None

            color = np.asanyarray(color_frame.get_data()).copy()
            depth = np.asanyarray(depth_frame.get_data()).copy()

        return color, depth, self._direct_camera_matrix

    def destroy_node(self):
        if self._rs_pipeline is not None:
            try:
                self._rs_pipeline.stop()
            except Exception:
                pass
        super().destroy_node()

    # ── Loaders ─────────────────────────────────────────────────────

    def _load_knowledge_base(self, path):
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f) or {}
            self.get_logger().info(f'Loaded knowledge base from {path}')
            return data
        except Exception as e:
            self.get_logger().warn(f'Could not load knowledge base ({path}): {e}')
            return {}

    def _ensure_model(self):
        if self._model is not None:
            return True
        if not self.model_path:
            self.get_logger().error('model_path parameter is empty - set it via launch')
            return False
        try:
            from ultralytics import YOLO
            self.get_logger().info(f'Loading YOLO model: {self.model_path}')
            self._model = YOLO(self.model_path)
            self.get_logger().info('YOLO model loaded')
            return True
        except ImportError:
            self.get_logger().error('ultralytics not installed: pip install ultralytics')
            return False
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            return False

    # ── Subscriber callbacks ───────────────────────────────────────

    def _on_color(self, msg):
        with self._lock:
            self._latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._latest_image_stamp = msg.header.stamp

    def _on_depth(self, msg):
        with self._lock:
            self._latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            self._latest_depth_stamp = msg.header.stamp

    def _on_camera_info(self, msg):
        with self._lock:
            self._camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self._dist_coeffs = (
                np.array(msg.d, dtype=np.float64) if len(msg.d) > 0
                else np.zeros((5,), dtype=np.float64)
            )

    # ── Service callback ───────────────────────────────────────────

    def _on_service(self, request, response):
        del request  # empty request
        try:
            result = self._build()
        except Exception as e:
            self.get_logger().error(f'World model build failed: {e}')
            response.success = False
            response.message = f'Exception: {e}'
            return response

        ok, msg, json_str, tools_msg, grasps_msg, annotated = result
        response.success = ok
        response.message = msg
        response.world_model_json = json_str
        if tools_msg is not None:
            response.tools_detected = tools_msg
        if grasps_msg is not None:
            response.grasp_candidates = grasps_msg

        if ok:
            self.json_pub.publish(String(data=json_str))
            if tools_msg is not None:
                self.tools_pub.publish(tools_msg)
            if grasps_msg is not None:
                self.grasp_pub.publish(grasps_msg)
        if annotated is not None:
            self.annotated_pub.publish(annotated)

        return response

    # ── Core build pipeline ────────────────────────────────────────

    def _build(self):
        # ── Acquire one frame, depending on camera_mode ──────────
        if self.camera_mode == 'direct':
            if self._rs_pipeline is None:
                return (
                    False, 'Direct RealSense pipeline not initialized',
                    '', None, None, None,
                )
            color, depth, cam_matrix = self._capture_direct()
            if color is None:
                return (
                    False, 'Direct capture returned no frame',
                    '', None, None, None,
                )
            cam_stamp = None  # always fresh
        else:
            with self._lock:
                color = None if self._latest_image is None else self._latest_image.copy()
                depth = None if self._latest_depth is None else self._latest_depth.copy()
                cam_matrix = self._camera_matrix
                cam_stamp = self._latest_image_stamp

            if color is None or cam_matrix is None:
                return False, 'No camera image / camera_info received yet', '', None, None, None

        now = self.get_clock().now().to_msg()
        if cam_stamp is not None:
            age = (now.sec - cam_stamp.sec) + (now.nanosec - cam_stamp.nanosec) * 1e-9
            if age > self.max_image_age_sec:
                return (
                    False,
                    f'Image stale ({age:.2f}s > {self.max_image_age_sec}s)',
                    '', None, None, None,
                )

        if not self._ensure_model():
            return False, 'YOLO model not available', '', None, None, None

        # Run inference
        try:
            results = self._model.predict(
                source=color,
                task='obb',
                imgsz=self.imgsz,
                conf=self.conf_threshold,
                device=self.device,
                verbose=False,
            )
        except Exception as e:
            return False, f'Inference failed: {e}', '', None, None, None

        result = results[0]
        names_map = result.names if hasattr(result, 'names') else {}
        detections = self._parse_results(result, names_map)

        # Pair handles with bodies
        bodies = [d for d in detections if d.cls_name != self.handle_class_name]
        handles = [d for d in detections if d.cls_name == self.handle_class_name]
        pairs = self._pair_handles_with_bodies(bodies, handles)
        unpaired_bodies = len(bodies) - len(pairs)

        # World-frame TF (may be unavailable if marker 120 not yet detected)
        tf_world_from_cam = self._lookup_tf_world_from_cam()
        world_available = tf_world_from_cam is not None

        # Build per-tool entries
        image_h, image_w = color.shape[:2]
        tools_msg = ToolDetectionArray()
        grasps_msg = GraspCandidateArray()
        tools_msg.header.stamp = now
        tools_msg.header.frame_id = self.tray_camera_frame
        grasps_msg.header.stamp = now
        grasps_msg.header.frame_id = self.world_frame if world_available else self.tray_camera_frame

        json_tools = []
        errors = []
        if unpaired_bodies > 0:
            errors.append(f'{unpaired_bodies} body OBB(s) without matching handle')
        if not world_available:
            errors.append(
                f'TF {self.world_frame} <- {self.tray_camera_frame} not available; '
                'world coordinates omitted (marker 120 may not be detected yet).'
            )

        annotated_img = color.copy()

        for body, handle in pairs:
            tool_id = f'tool_{self._tool_seq}'
            self._tool_seq += 1

            # Long axis of body
            long_angle, long_length = _obb_long_axis(body.width, body.height, body.angle_rad)
            axis_unit = np.array([math.cos(long_angle), math.sin(long_angle)])

            # Sign of handle position along long axis (from body center)
            handle_proj = _project_onto_axis(body.center, handle.center, axis_unit)
            sign = 1.0 if handle_proj >= 0.0 else -1.0
            handle_side = 'positive' if sign > 0 else 'negative'

            # Grasp point: from handle center, move toward body center along axis
            offset_px = self.grasp_offset_fraction * float(image_h)
            grasp_px = handle.center - sign * offset_px * axis_unit
            grasp_px = np.clip(
                grasp_px, [0.0, 0.0], [float(image_w - 1), float(image_h - 1)]
            )

            # 3D projection
            grasp_camera_xyz = self._pixel_to_camera(grasp_px[0], grasp_px[1], depth, cam_matrix)
            grasp_world_xyz = None
            long_axis_world = None
            handle_dir_world = None
            functional_end_world = None
            grasp_rot_z = None

            use_fixed_plane = math.isfinite(self.fixed_tool_plane_z_m)
            if tf_world_from_cam is not None:
                if use_fixed_plane:
                    grasp_world_xyz = self._pixel_to_world_on_plane(
                        grasp_px[0], grasp_px[1], cam_matrix,
                        tf_world_from_cam, self.fixed_tool_plane_z_m,
                    )
                elif grasp_camera_xyz is not None:
                    grasp_world_xyz = self._transform(tf_world_from_cam, grasp_camera_xyz)

            if grasp_world_xyz is not None and tf_world_from_cam is not None:
                # Sample two pixels along the long axis to derive its world direction
                p1 = body.center
                p2 = body.center + 0.25 * long_length * axis_unit
                if use_fixed_plane:
                    p1_w = self._pixel_to_world_on_plane(
                        p1[0], p1[1], cam_matrix,
                        tf_world_from_cam, self.fixed_tool_plane_z_m,
                    )
                    p2_w = self._pixel_to_world_on_plane(
                        p2[0], p2[1], cam_matrix,
                        tf_world_from_cam, self.fixed_tool_plane_z_m,
                    )
                else:
                    p1_cam = self._pixel_to_camera(p1[0], p1[1], depth, cam_matrix)
                    p2_cam = self._pixel_to_camera(p2[0], p2[1], depth, cam_matrix)
                    p1_w = self._transform(tf_world_from_cam, p1_cam) if p1_cam is not None else None
                    p2_w = self._transform(tf_world_from_cam, p2_cam) if p2_cam is not None else None

                if p1_w is not None and p2_w is not None:
                    direction = p2_w - p1_w
                    direction[2] = 0.0
                    n = np.linalg.norm(direction)
                    if n > 1e-6:
                        long_axis_world = direction / n
                        # Handle direction = sign * long_axis_world
                        handle_dir_world = sign * long_axis_world
                        functional_end_world = -handle_dir_world
                        grasp_rot_z = math.atan2(long_axis_world[1], long_axis_world[0])

            # Semantics from knowledge base
            kb_entry = self._lookup_tool_kb(body.cls_name)

            # Whether handover requires a 180 deg flip:
            # "negative" = handle on negative side of body center; flip flag is a
            # simple boolean for downstream execution to decide gripper orientation.
            needs_flip = (handle_side == 'positive')

            # ── Build dict for JSON ──
            tool_dict = {
                'tool_id': tool_id,
                'tool_class': body.cls_name,
                'display_name': kb_entry.get('display_name', body.cls_name),
                'confidence': body.conf,
                'handle_confidence': handle.conf,
                'body_obb_px': body.to_dict(),
                'handle_obb_px': handle.to_dict(),
                'long_axis': {
                    'angle_rad_image': float(long_angle),
                    'length_px': float(long_length),
                    'handle_side': handle_side,
                },
                'grasp': {
                    'point_px': grasp_px.tolist(),
                    'point_camera_m': (grasp_camera_xyz.tolist()
                                       if grasp_camera_xyz is not None else None),
                    'point_world_m': (grasp_world_xyz.tolist()
                                      if grasp_world_xyz is not None else None),
                    'projection_mode': (
                        f'fixed_world_z={self.fixed_tool_plane_z_m:.4f}'
                        if use_fixed_plane else 'realsense_depth'
                    ),
                    'rotation_world_z_rad': grasp_rot_z,
                    'needs_180_flip_for_handover': needs_flip,
                    'approach_direction_world': [0.0, 0.0, -1.0],  # top-down default
                },
                'directions': {
                    'long_axis_world': (long_axis_world.tolist()
                                        if long_axis_world is not None else None),
                    'handle_direction_world': (handle_dir_world.tolist()
                                               if handle_dir_world is not None else None),
                    'functional_end_direction_world': (
                        functional_end_world.tolist()
                        if functional_end_world is not None else None),
                },
                'semantics': {
                    'handover_rule': kb_entry.get('handover_rule', 'neutral'),
                    'handover_description': kb_entry.get('handover_description', ''),
                    'functional_end_label': kb_entry.get('functional_end', ''),
                    'lift_height_m': kb_entry.get('lift_height', 0.10),
                    'grip_strategy': kb_entry.get('grip_strategy', 'parallel'),
                },
            }
            json_tools.append(tool_dict)

            # ── Build ROS messages ──
            tools_msg.detections.append(
                self._build_tool_detection_msg(
                    body, handle, tool_id,
                    body_3d=self._pixel_to_world_safe(body.center, depth, cam_matrix, tf_world_from_cam),
                    handle_3d=self._pixel_to_world_safe(handle.center, depth, cam_matrix, tf_world_from_cam),
                )
            )
            if grasp_world_xyz is not None:
                grasps_msg.candidates.append(
                    self._build_grasp_candidate_msg(
                        body, handle, tool_id,
                        grasp_world=grasp_world_xyz,
                        grasp_rot_z=grasp_rot_z,
                        long_axis_world=long_axis_world,
                        handle_dir_world=handle_dir_world,
                        functional_end_world=functional_end_world,
                        kb_entry=kb_entry,
                    )
                )
            else:
                errors.append(
                    f'{tool_id} ({body.cls_name}) has no valid world grasp point; '
                    'skipping ROS GraspCandidate.'
                )

            # ── Annotate image ──
            self._annotate(annotated_img, body, handle, grasp_px, long_angle, sign, tool_id)

        # ── Build JSON envelope ──
        json_payload = {
            'timestamp_sec': now.sec + now.nanosec * 1e-9,
            'tray_camera_frame': self.tray_camera_frame,
            'world_frame': self.world_frame,
            'world_frame_available': world_available,
            'image_size_px': [int(image_w), int(image_h)],
            'tools': json_tools,
            'errors': errors,
        }
        json_str = json.dumps(json_payload, indent=2)

        # ── Annotated image message ──
        ann_msg = self.bridge.cv2_to_imgmsg(annotated_img, encoding='bgr8')
        ann_msg.header.stamp = now
        ann_msg.header.frame_id = self.tray_camera_frame

        n_tools = len(json_tools)
        return (
            True,
            f'{n_tools} tool(s) detected' + (f'; {len(errors)} warning(s)' if errors else ''),
            json_str, tools_msg, grasps_msg, ann_msg,
        )

    # ── Helpers ────────────────────────────────────────────────────

    def _parse_results(self, result, names_map):
        out = []
        if result.obb is None or result.obb.xyxyxyxy is None:
            return out
        try:
            xyxyxyxy = result.obb.xyxyxyxy.cpu().numpy()
            xywhr = result.obb.xywhr.cpu().numpy()
            cls = result.obb.cls.cpu().numpy().astype(int)
            conf = result.obb.conf.cpu().numpy()
        except Exception as e:
            self.get_logger().warn(f'Failed to read OBB tensors: {e}')
            return out

        for pts, xywhr_i, cls_i, conf_i in zip(xyxyxyxy, xywhr, cls, conf):
            cls_name = names_map.get(int(cls_i), str(int(cls_i)))
            cx, cy, w, h, r = xywhr_i
            out.append(_Detection(
                cls_name=cls_name, conf=float(conf_i),
                center=(float(cx), float(cy)),
                w=float(w), h=float(h), angle_rad=float(r),
                corners=pts,
            ))
        return out

    def _pair_handles_with_bodies(self, bodies, handles):
        """For each body, pick the highest-confidence handle whose center is inside it."""
        pairs = []
        used = set()
        for body in bodies:
            best = None
            for i, handle in enumerate(handles):
                if i in used:
                    continue
                if not _point_in_obb(handle.center, body.corners):
                    continue
                if best is None or handle.conf > best[1].conf:
                    best = (i, handle)
            if best is not None:
                used.add(best[0])
                pairs.append((body, best[1]))
            else:
                self.get_logger().info(
                    f'No handle inside body OBB for class "{body.cls_name}"'
                )
        return pairs

    def _pixel_to_camera(self, u, v, depth_img, cam_matrix):
        if depth_img is None or cam_matrix is None:
            return None
        ui, vi = int(round(u)), int(round(v))
        if not (0 <= vi < depth_img.shape[0] and 0 <= ui < depth_img.shape[1]):
            return None
        z = self._median_depth_m(depth_img, ui, vi)
        if z is None:
            return None
        fx = cam_matrix[0, 0]
        fy = cam_matrix[1, 1]
        cx = cam_matrix[0, 2]
        cy = cam_matrix[1, 2]
        x = (u - cx) / fx * z
        y = (v - cy) / fy * z
        return np.array([x, y, z], dtype=float)

    def _median_depth_m(self, depth_img, ui, vi):
        radius = max(0, int(self.depth_search_radius_px))
        h, w = depth_img.shape[:2]
        u0 = max(0, ui - radius)
        u1 = min(w, ui + radius + 1)
        v0 = max(0, vi - radius)
        v1 = min(h, vi + radius + 1)

        window = np.asarray(depth_img[v0:v1, u0:u1], dtype=np.float64) / 1000.0
        valid = window[
            np.isfinite(window)
            & (window > 0.0)
            & (window >= self.depth_min_m)
            & (window <= self.depth_max_m)
        ]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def _lookup_tf_world_from_cam(self):
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.world_frame, self.tray_camera_frame, rclpy.time.Time()
            )
            return _tf_to_matrix(tf_msg)
        except Exception:
            return None

    @staticmethod
    def _transform(T_world_from_cam, p_cam):
        p_h = np.array([p_cam[0], p_cam[1], p_cam[2], 1.0])
        p_w = T_world_from_cam @ p_h
        return p_w[:3]

    def _pixel_to_world_safe(self, pt_px, depth_img, cam_matrix, tf_world_from_cam):
        if tf_world_from_cam is None:
            return None
        if math.isfinite(self.fixed_tool_plane_z_m):
            return self._pixel_to_world_on_plane(
                pt_px[0], pt_px[1], cam_matrix,
                tf_world_from_cam, self.fixed_tool_plane_z_m,
            )
        cam_pt = self._pixel_to_camera(pt_px[0], pt_px[1], depth_img, cam_matrix)
        if cam_pt is None:
            return None
        return self._transform(tf_world_from_cam, cam_pt)

    def _pixel_to_world_on_plane(self, u, v, cam_matrix, tf_world_from_cam, plane_z_world):
        if cam_matrix is None or tf_world_from_cam is None:
            return None

        fx = cam_matrix[0, 0]
        fy = cam_matrix[1, 1]
        cx = cam_matrix[0, 2]
        cy = cam_matrix[1, 2]
        if abs(fx) < 1e-9 or abs(fy) < 1e-9:
            return None

        ray_cam = np.array([(u - cx) / fx, (v - cy) / fy, 1.0], dtype=float)
        ray_cam_norm = np.linalg.norm(ray_cam)
        if ray_cam_norm < 1e-9:
            return None
        ray_cam = ray_cam / ray_cam_norm

        origin_world = tf_world_from_cam[:3, 3]
        ray_world = tf_world_from_cam[:3, :3] @ ray_cam
        ray_world_norm = np.linalg.norm(ray_world)
        if ray_world_norm < 1e-9 or abs(ray_world[2]) < 1e-9:
            return None
        ray_world = ray_world / ray_world_norm

        t = (plane_z_world - origin_world[2]) / ray_world[2]
        if t <= 0.0:
            return None
        return origin_world + t * ray_world

    def _lookup_tool_kb(self, tool_class):
        tools = self.knowledge_base.get('tools', {})
        if tool_class in tools:
            return tools[tool_class]
        # Synonym match
        for cls_name, info in tools.items():
            if tool_class.lower() in [s.lower() for s in info.get('synonyms', [])]:
                return info
        return self.knowledge_base.get('default', {}) or {}

    def _build_tool_detection_msg(self, body, handle, tool_id, body_3d, handle_3d):
        msg = ToolDetection()
        msg.tool_id = tool_id
        msg.tool_class = body.cls_name
        msg.confidence = body.conf
        msg.body_obb = body.to_msg()
        msg.handle_obb = handle.to_msg()
        if body_3d is not None and handle_3d is not None:
            msg.body_center_3d = Point(
                x=float(body_3d[0]), y=float(body_3d[1]), z=float(body_3d[2])
            )
            msg.handle_center_3d = Point(
                x=float(handle_3d[0]), y=float(handle_3d[1]), z=float(handle_3d[2])
            )
            msg.has_3d = True
        else:
            msg.has_3d = False
        return msg

    def _build_grasp_candidate_msg(self, body, handle, tool_id,
                                   grasp_world, grasp_rot_z,
                                   long_axis_world, handle_dir_world,
                                   functional_end_world, kb_entry):
        msg = GraspCandidate()
        msg.tool_id = tool_id
        msg.tool_class = body.cls_name
        msg.grasp_confidence = float(min(body.conf, handle.conf))
        msg.handover_rule = kb_entry.get('handover_rule', 'neutral')

        # grasp_pose: position + orientation (rotation around world Z) in world
        msg.grasp_pose.header.frame_id = self.world_frame
        msg.grasp_pose.header.stamp = self.get_clock().now().to_msg()
        if grasp_world is not None:
            msg.grasp_pose.pose.position.x = float(grasp_world[0])
            msg.grasp_pose.pose.position.y = float(grasp_world[1])
            msg.grasp_pose.pose.position.z = float(grasp_world[2])
        if grasp_rot_z is not None:
            quat = R.from_euler('z', grasp_rot_z).as_quat()
            msg.grasp_pose.pose.orientation.x = float(quat[0])
            msg.grasp_pose.pose.orientation.y = float(quat[1])
            msg.grasp_pose.pose.orientation.z = float(quat[2])
            msg.grasp_pose.pose.orientation.w = float(quat[3])
        else:
            msg.grasp_pose.pose.orientation.w = 1.0

        # Approach direction default: top-down
        msg.approach_direction = Vector3(x=0.0, y=0.0, z=-1.0)

        if functional_end_world is not None:
            msg.functional_end_dir = Vector3(
                x=float(functional_end_world[0]),
                y=float(functional_end_world[1]),
                z=float(functional_end_world[2]),
            )
        if long_axis_world is not None:
            msg.handle_axis = Vector3(
                x=float(long_axis_world[0]),
                y=float(long_axis_world[1]),
                z=float(long_axis_world[2]),
            )

        # handover_pose left empty - filled later by execution layer when hand is known
        msg.handover_pose.header.frame_id = self.world_frame
        msg.handover_pose.pose.orientation.w = 1.0
        return msg

    def _annotate(self, img, body, handle, grasp_px, long_angle, sign, tool_id):
        # Body OBB in green
        cv2.polylines(img, [body.corners.astype(np.int32)], True, (0, 255, 0), 2, cv2.LINE_AA)
        # Handle OBB in blue
        cv2.polylines(img, [handle.corners.astype(np.int32)], True, (255, 180, 0), 2, cv2.LINE_AA)
        # Long axis line
        axis_unit = np.array([math.cos(long_angle), math.sin(long_angle)])
        p_a = (body.center - 0.5 * max(body.width, body.height) * axis_unit).astype(int)
        p_b = (body.center + 0.5 * max(body.width, body.height) * axis_unit).astype(int)
        cv2.line(img, tuple(p_a), tuple(p_b), (0, 255, 255), 1, cv2.LINE_AA)
        # Grasp point in red
        gp = tuple(grasp_px.astype(int))
        cv2.circle(img, gp, 7, (0, 0, 255), -1, cv2.LINE_AA)
        cv2.putText(
            img, f'{tool_id} {body.cls_name}', (gp[0] + 10, gp[1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA,
        )


def main(args=None):
    rclpy.init(args=args)
    node = WorldModelBuilder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
