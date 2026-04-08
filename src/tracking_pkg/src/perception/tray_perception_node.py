#!/usr/bin/env python3
"""
Tray Perception Node
====================
Subscribes to the tray camera (RGB + Depth), runs OBB inference on each frame,
and publishes ToolDetectionArray with 3D-projected positions in world frame.

Pipeline:
  tray_camera/color_image → OBB model inference → pixel OBBs
  tray_camera/depth_image → depth lookup per OBB center
  TF (tray_camera_frame → world) → 3D projection

Publishes:
  /tools_detected (tracking_pkg/msg/ToolDetectionArray)

Parameters:
  model_path (str): Path to OBB model weights
  confidence_threshold (float): Minimum detection confidence
  inference_rate_hz (float): Max inference rate
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import tf2_ros

# Message imports will work after colcon build + source
# from tracking_pkg.msg import ToolDetection, ToolDetectionArray, OrientedBoundingBox2D


class TrayPerceptionNode(Node):
    def __init__(self):
        super().__init__('tray_perception_node')

        # Parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('inference_rate_hz', 10.0)

        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        inference_hz = self.get_parameter('inference_rate_hz').value

        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None
        self.camera_matrix = None

        # TF for 3D projection
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers - tray camera topics
        self.create_subscription(Image, 'tray_camera/color_image', self._rgb_cb, 10)
        self.create_subscription(Image, 'tray_camera/depth_image', self._depth_cb, 10)
        self.create_subscription(CameraInfo, 'tray_camera/camera_info', self._cam_info_cb, 10)

        # Publisher
        # self.detection_pub = self.create_publisher(ToolDetectionArray, 'tools_detected', 10)

        # Inference timer
        period = 1.0 / inference_hz if inference_hz > 0 else 0.1
        self.create_timer(period, self._inference_tick)

        # OBB model (loaded lazily)
        self.model = None

        self.get_logger().info(f'TrayPerceptionNode initialized (model: {self.model_path})')

    def _rgb_cb(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _depth_cb(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')

    def _cam_info_cb(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)

    def _load_model(self):
        """Load OBB detection model. Override with actual model loading."""
        if not self.model_path:
            self.get_logger().warn('No model_path set, skipping model load')
            return False
        # TODO: Load YOLO-OBB or custom OBB model
        # from ultralytics import YOLO
        # self.model = YOLO(self.model_path)
        self.get_logger().info(f'Model loaded from {self.model_path}')
        return True

    def _inference_tick(self):
        """Run OBB inference on current frame."""
        if self.rgb_image is None:
            return

        if self.model is None:
            if not self._load_model():
                return

        # TODO: Implement inference pipeline
        # 1. Run OBB model on self.rgb_image
        # 2. For each detection: extract class, confidence, OBB corners
        # 3. Separate handle vs body OBBs (by class suffix or separate model heads)
        # 4. Project OBB centers to 3D using depth + camera intrinsics + TF
        # 5. Publish ToolDetectionArray
        pass

    def _pixel_to_world(self, u, v):
        """Project pixel (u,v) + depth to world frame coordinates."""
        if self.depth_image is None or self.camera_matrix is None:
            return None

        depth = self.depth_image[int(v), int(u)] / 1000.0  # mm to m
        if depth <= 0.0:
            return None

        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        x_cam = (u - cx) / fx * depth
        y_cam = (v - cy) / fy * depth
        z_cam = depth

        # Transform camera → world via TF
        try:
            tf = self.tf_buffer.lookup_transform('world', 'tray_camera_frame', rclpy.time.Time())
            # TODO: Apply transform to [x_cam, y_cam, z_cam]
            return None
        except Exception:
            return None


def main(args=None):
    rclpy.init(args=args)
    node = TrayPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
