#!/usr/bin/env python3
"""
Tool Detection Node
===================
Subscribes to the tray camera (color + aligned depth), runs YOLOv8-OBB
inference at a fixed rate, pairs body and handle OBBs per tool, projects
body/handle centers to the world frame, and publishes a ToolDetectionArray
on /detected_tools_obb.

This is the perception stage of the Phase 1 LLM architecture. It does NOT
compute grasp points, tool axes, or semantics — those belong to downstream
nodes (grasp_geometry_node, tool_semantics_node, world_model_node).

Topics in:
  {tray_camera_namespace}/color/image_raw                 (sensor_msgs/Image)
  {tray_camera_namespace}/aligned_depth_to_color/image_raw (sensor_msgs/Image)
  {tray_camera_namespace}/color/camera_info               (sensor_msgs/CameraInfo)

Topics out:
  /detected_tools_obb              (tracking_pkg/ToolDetectionArray)
  /tool_detection/annotated_image  (sensor_msgs/Image, debug)
"""

import math
from threading import Lock

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Point32
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros

from tracking_pkg.msg import (
    OrientedBoundingBox2D,
    ToolDetection,
    ToolDetectionArray,
)


def _tf_to_matrix(tf_msg):
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    rot = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = [t.x, t.y, t.z]
    return T


def _point_in_obb(pt, corners):
    poly = np.asarray(corners, dtype=np.float32).reshape(-1, 2)
    return cv2.pointPolygonTest(poly, (float(pt[0]), float(pt[1])), False) >= 0.0


class _Detection:
    def __init__(self, cls_name, conf, center, w, h, angle_rad, corners):
        self.cls_name = cls_name
        self.conf = float(conf)
        self.center = np.array(center, dtype=float)
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


class ToolDetectionNode(Node):
    def __init__(self):
        super().__init__('tool_detection_node')

        self.declare_parameter('model_path', '')
        self.declare_parameter('tray_camera_namespace', '/tray_camera')
        self.declare_parameter('tray_camera_frame', 'tray_camera_color_optical_frame')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('conf_threshold', 0.35)
        self.declare_parameter('imgsz', 1024)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('handle_class_name', 'handle')
        self.declare_parameter('inference_rate_hz', 5.0)
        self.declare_parameter('depth_search_radius_px', 5)
        self.declare_parameter('depth_min_m', 0.05)
        self.declare_parameter('depth_max_m', 2.0)
        self.declare_parameter('fixed_tool_plane_z_m', 0.030)
        self.declare_parameter('publish_annotated_image', True)

        self.model_path = self.get_parameter('model_path').value
        ns = self.get_parameter('tray_camera_namespace').value.rstrip('/')
        self.tray_camera_frame = self.get_parameter('tray_camera_frame').value
        self.world_frame = self.get_parameter('world_frame').value
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)
        self.imgsz = int(self.get_parameter('imgsz').value)
        self.device = self.get_parameter('device').value
        self.handle_class_name = self.get_parameter('handle_class_name').value
        self.inference_rate_hz = float(self.get_parameter('inference_rate_hz').value)
        self.depth_search_radius_px = int(self.get_parameter('depth_search_radius_px').value)
        self.depth_min_m = float(self.get_parameter('depth_min_m').value)
        self.depth_max_m = float(self.get_parameter('depth_max_m').value)
        self.fixed_tool_plane_z_m = float(self.get_parameter('fixed_tool_plane_z_m').value)
        self.publish_annotated_image = bool(self.get_parameter('publish_annotated_image').value)

        self.bridge = CvBridge()
        self._lock = Lock()
        self._latest_image = None
        self._latest_depth = None
        self._camera_matrix = None

        self._model = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(Image, f'{ns}/color/image_raw', self._on_color, 10)
        self.create_subscription(
            Image, f'{ns}/aligned_depth_to_color/image_raw', self._on_depth, 10
        )
        self.create_subscription(
            CameraInfo, f'{ns}/color/camera_info', self._on_camera_info, 10
        )

        self.tools_pub = self.create_publisher(
            ToolDetectionArray, '/detected_tools_obb', 10
        )
        self.annotated_pub = (
            self.create_publisher(Image, '/tool_detection/annotated_image', 10)
            if self.publish_annotated_image else None
        )

        period = 1.0 / self.inference_rate_hz if self.inference_rate_hz > 0 else 0.2
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f'ToolDetectionNode ready (ns={ns}, model={self.model_path or "<unset>"}, '
            f'rate={self.inference_rate_hz} Hz, device={self.device})'
        )

    def _on_color(self, msg):
        with self._lock:
            self._latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _on_depth(self, msg):
        with self._lock:
            self._latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')

    def _on_camera_info(self, msg):
        with self._lock:
            self._camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)

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

    def _tick(self):
        with self._lock:
            color = None if self._latest_image is None else self._latest_image.copy()
            depth = None if self._latest_depth is None else self._latest_depth.copy()
            cam_matrix = self._camera_matrix

        if color is None or cam_matrix is None:
            return

        now_msg = self.get_clock().now().to_msg()

        if not self._ensure_model():
            return

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
            self.get_logger().error(f'Inference failed: {e}')
            return

        result = results[0]
        names_map = result.names if hasattr(result, 'names') else {}
        detections = self._parse_results(result, names_map)

        bodies = [d for d in detections if d.cls_name != self.handle_class_name]
        handles = [d for d in detections if d.cls_name == self.handle_class_name]
        pairs = self._pair_handles_with_bodies(bodies, handles)

        tf_world_from_cam = self._lookup_tf_world_from_cam()

        out = ToolDetectionArray()
        out.header.stamp = now_msg
        out.header.frame_id = self.tray_camera_frame

        annotated = color.copy() if self.annotated_pub is not None else None

        for idx, (body, handle) in enumerate(pairs):
            tool_id = f'tool_{idx}'
            body_world = self._pixel_to_world(body.center, depth, cam_matrix, tf_world_from_cam)
            handle_world = self._pixel_to_world(handle.center, depth, cam_matrix, tf_world_from_cam)

            det = ToolDetection()
            det.header = out.header
            det.tool_id = tool_id
            det.tool_class = body.cls_name
            det.confidence = body.conf
            det.body_obb = body.to_msg()
            det.handle_obb = handle.to_msg()
            if body_world is not None and handle_world is not None:
                det.body_center_3d = Point(
                    x=float(body_world[0]), y=float(body_world[1]), z=float(body_world[2]),
                )
                det.handle_center_3d = Point(
                    x=float(handle_world[0]), y=float(handle_world[1]), z=float(handle_world[2]),
                )
                det.has_3d = True
            else:
                det.has_3d = False
            out.detections.append(det)

            if annotated is not None:
                self._annotate(annotated, body, handle, tool_id)

        self.tools_pub.publish(out)

        if annotated is not None:
            ann_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            ann_msg.header.stamp = now_msg
            ann_msg.header.frame_id = self.tray_camera_frame
            self.annotated_pub.publish(ann_msg)

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
        return pairs

    def _lookup_tf_world_from_cam(self):
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.world_frame, self.tray_camera_frame, rclpy.time.Time()
            )
            return _tf_to_matrix(tf_msg)
        except Exception:
            return None

    # NOTE: Currently using fixed-plane projection because the RealSense is
    # mounted closer to the tray than its minimum sensor depth, so depth
    # readings on the tools are unreliable. Once the camera is repositioned,
    # set fixed_tool_plane_z_m to NaN to revert to depth-based projection.
    def _pixel_to_world(self, pt_px, depth_img, cam_matrix, tf_world_from_cam):
        if tf_world_from_cam is None or cam_matrix is None:
            return None

        # Fixed-plane mode: ray-plane intersection at z = fixed_tool_plane_z_m
        if math.isfinite(self.fixed_tool_plane_z_m):
            return self._pixel_to_world_on_plane(
                pt_px[0], pt_px[1], cam_matrix,
                tf_world_from_cam, self.fixed_tool_plane_z_m,
            )

        # Depth-based mode (fallback when camera is repositioned)
        if depth_img is None:
            return None
        ui, vi = int(round(pt_px[0])), int(round(pt_px[1]))
        if not (0 <= vi < depth_img.shape[0] and 0 <= ui < depth_img.shape[1]):
            return None
        z = self._median_depth_m(depth_img, ui, vi)
        if z is None:
            return None
        fx = cam_matrix[0, 0]
        fy = cam_matrix[1, 1]
        cx = cam_matrix[0, 2]
        cy = cam_matrix[1, 2]
        if abs(fx) < 1e-9 or abs(fy) < 1e-9:
            return None
        x_cam = (pt_px[0] - cx) / fx * z
        y_cam = (pt_px[1] - cy) / fy * z
        p_cam = np.array([x_cam, y_cam, z, 1.0])
        p_world = tf_world_from_cam @ p_cam
        return p_world[:3]

    def _pixel_to_world_on_plane(self, u, v, cam_matrix, tf_world_from_cam, plane_z_world):
        """Ray-plane intersection: shoot ray through pixel, intersect z=plane plane in world."""
        fx = cam_matrix[0, 0]
        fy = cam_matrix[1, 1]
        cx = cam_matrix[0, 2]
        cy = cam_matrix[1, 2]
        if abs(fx) < 1e-9 or abs(fy) < 1e-9:
            return None

        ray_cam = np.array([(u - cx) / fx, (v - cy) / fy, 1.0], dtype=float)
        ray_cam_norm = float(np.linalg.norm(ray_cam))
        if ray_cam_norm < 1e-9:
            return None
        ray_cam = ray_cam / ray_cam_norm

        origin_world = tf_world_from_cam[:3, 3]
        ray_world = tf_world_from_cam[:3, :3] @ ray_cam
        ray_world_norm = float(np.linalg.norm(ray_world))
        if ray_world_norm < 1e-9 or abs(ray_world[2]) < 1e-9:
            return None
        ray_world = ray_world / ray_world_norm

        t = (plane_z_world - origin_world[2]) / ray_world[2]
        if t <= 0.0:
            return None
        return origin_world + t * ray_world

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

    def _annotate(self, img, body, handle, tool_id):
        cv2.polylines(img, [body.corners.astype(np.int32)], True, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.polylines(img, [handle.corners.astype(np.int32)], True, (255, 180, 0), 2, cv2.LINE_AA)
        center = body.center.astype(int)
        cv2.putText(
            img, f'{tool_id} {body.cls_name} {body.conf:.2f}',
            (int(center[0]) + 8, int(center[1]) - 8),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA,
        )


def main(args=None):
    rclpy.init(args=args)
    node = ToolDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
