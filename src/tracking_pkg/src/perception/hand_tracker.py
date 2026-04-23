#!/usr/bin/env python3
"""
Hand Tracker Node (Refactored)
==============================
Continuous hand keypoint publisher using MediaPipe.
No gesture logic - publishes hand pose continuously on every tracked frame.

Replaces the old gesture-triggered hand_tracker that only published
on double-open-close or shaka gestures.

Subscriptions:
  /color_image (sensor_msgs/Image) - RGB from scene camera
  /depth_image (sensor_msgs/Image) - Depth from scene camera
  /camera_info (sensor_msgs/CameraInfo) - Camera intrinsics

Publishers:
  /hand_state (tracking_pkg/msg/HandState) - continuous hand pose + keypoints
  /hand_pose (geometry_msgs/Pose) - legacy compat, continuous publish
  /hand_pose_marker (visualization_msgs/Marker) - RViz visualization
  /annotated_hand_image (sensor_msgs/Image) - debug visualization

Parameters:
  max_num_hands (int): Max hands to detect (default: 2)
  min_detection_confidence (float): MediaPipe detection threshold
  min_tracking_confidence (float): MediaPipe tracking threshold
  publish_rate_hz (float): Max publish rate for hand state
  annotated_image_max_hz (float): Max rate for annotated image
"""

import cv2
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from threading import Thread

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, Point, PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from cv_bridge import CvBridge
import tf2_ros
import mediapipe as mp

# Will be available after first build + source with new messages:
# from tracking_pkg.msg import HandState


class HandTrackerNode(Node):
    def __init__(self):
        super().__init__('hand_tracker')

        # Parameters
        self.declare_parameter('max_num_hands', 2)
        self.declare_parameter('min_detection_confidence', 0.5)
        self.declare_parameter('min_tracking_confidence', 0.5)
        self.declare_parameter('publish_rate_hz', 15.0)
        self.declare_parameter('annotated_image_max_hz', 12.0)
        self.declare_parameter('camera_frame', 'scene_camera_color_optical_frame')
        self.declare_parameter('world_frame', 'world')

        max_hands = int(self.get_parameter('max_num_hands').value)
        det_conf = float(self.get_parameter('min_detection_confidence').value)
        track_conf = float(self.get_parameter('min_tracking_confidence').value)
        pub_hz = float(self.get_parameter('publish_rate_hz').value)
        ann_hz = float(self.get_parameter('annotated_image_max_hz').value)
        self.camera_frame = self.get_parameter('camera_frame').value
        self.world_frame = self.get_parameter('world_frame').value

        self.publish_interval = 1.0 / pub_hz if pub_hz > 0 else 0.0
        self.annotated_interval = 1.0 / ann_hz if ann_hz > 0 else 0.0

        # MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=max_hands,
            min_detection_confidence=det_conf,
            min_tracking_confidence=track_conf,
        )
        self.mp_drawing = mp.solutions.drawing_utils

        # State
        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None
        self.camera_matrix = None
        self.last_publish_time = 0.0
        self.last_annotated_time = 0.0
        self.last_tf_warn_time = 0.0

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers
        self.create_subscription(Image, 'color_image', self._rgb_cb, 10)
        self.create_subscription(Image, 'depth_image', self._depth_cb, 10)
        self.create_subscription(CameraInfo, 'camera_info', self._cam_info_cb, 10)

        # Publishers
        # TODO: Uncomment after first build with new messages
        # self.hand_state_pub = self.create_publisher(HandState, 'hand_state', 10)
        self.hand_pose_pub = self.create_publisher(Pose, 'hand_pose', 10)
        self.marker_pub = self.create_publisher(Marker, 'hand_pose_marker', 10)
        self.annotated_pub = self.create_publisher(Image, 'annotated_hand_image', 10)

        self.get_logger().info('HandTrackerNode initialized (continuous mode, no gestures)')

    # ── Callbacks ──────────────────────────────────────────────────

    def _rgb_cb(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _depth_cb(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')

    def _cam_info_cb(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)

    # ── Coordinate transforms ─────────────────────────────────────

    def _pixel_to_camera(self, u, v, depth_m):
        """Project pixel (u,v) + depth to camera-frame 3D point."""
        if self.camera_matrix is None:
            return None
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        x = (u - cx) / fx * depth_m
        y = (v - cy) / fy * depth_m
        return np.array([x, y, depth_m])

    def _camera_to_world(self, cam_coords):
        """Transform camera-frame point to world frame via single TF lookup.

        TF chain (produced by aruco_marker_manager + launch):
          world -> base -> aruco_marker_100_frame -> scene_camera_color_optical_frame
        tf2 collapses this into a single lookup.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame, self.camera_frame, rclpy.time.Time())
            T = self._tf_to_matrix(tf)

            pt = np.array([cam_coords[0], cam_coords[1], cam_coords[2], 1.0])
            world_pt = T @ pt
            return world_pt[:3]
        except Exception as e:
            now = time.time()
            if now - self.last_tf_warn_time > 2.0:
                self.get_logger().warn(
                    f'TF unavailable ({self.world_frame} <- {self.camera_frame}): {e}'
                )
                self.last_tf_warn_time = now
            return None

    def _tf_to_matrix(self, tf_msg):
        """Convert TransformStamped to 4x4 homogeneous matrix."""
        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        rot = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T = np.eye(4)
        T[:3, :3] = rot
        T[:3, 3] = [t.x, t.y, t.z]
        return T

    # ── Landmark helpers ──────────────────────────────────────────

    def _get_palm_center_px(self, hand_landmarks, frame_shape):
        """Compute palm center pixel from wrist + middle_mcp average."""
        wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
        mid_mcp = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP]
        h, w = frame_shape[:2]
        cx = int((wrist.x + mid_mcp.x) / 2 * w)
        cy = int((wrist.y + mid_mcp.y) / 2 * h)
        return cx, cy

    def _landmark_to_world(self, landmark, frame_shape):
        """Project a single MediaPipe landmark to world frame 3D point."""
        if self.depth_image is None:
            return None
        h, w = frame_shape[:2]
        px = int(landmark.x * w)
        py = int(landmark.y * h)
        if not (0 <= py < self.depth_image.shape[0] and 0 <= px < self.depth_image.shape[1]):
            return None
        depth_m = self.depth_image[py, px] / 1000.0
        if depth_m <= 0.0:
            return None
        cam_pt = self._pixel_to_camera(px, py, depth_m)
        if cam_pt is None:
            return None
        return self._camera_to_world(cam_pt)

    # ── Main loop ─────────────────────────────────────────────────

    def run(self):
        """Main processing loop: detect hands, publish continuously."""
        while rclpy.ok():
            if self.rgb_image is None or self.depth_image is None:
                time.sleep(0.05)
                continue

            now = time.time()
            frame = self.rgb_image.copy()
            depth = self.depth_image.copy()

            # MediaPipe inference
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.hands.process(rgb)

            if results.multi_hand_landmarks:
                for hand_lm in results.multi_hand_landmarks:
                    self.mp_drawing.draw_landmarks(
                        frame, hand_lm, self.mp_hands.HAND_CONNECTIONS)

                # Track closest / first hand for now
                # TODO: Multi-hand tracking with persistent IDs
                best_hand = results.multi_hand_landmarks[0]
                cx, cy = self._get_palm_center_px(best_hand, frame.shape)

                if (0 <= cy < depth.shape[0] and 0 <= cx < depth.shape[1]):
                    depth_m = depth[cy, cx] / 1000.0

                    if depth_m > 0.0 and now - self.last_publish_time >= self.publish_interval:
                        cam_pt = self._pixel_to_camera(cx, cy, depth_m)
                        if cam_pt is not None:
                            world_pt = self._camera_to_world(cam_pt)
                            if world_pt is not None:
                                # Continuous hand pose publish
                                pose_msg = Pose()
                                pose_msg.position.x = float(world_pt[0])
                                pose_msg.position.y = float(world_pt[1])
                                pose_msg.position.z = float(world_pt[2])
                                pose_msg.orientation.w = 1.0
                                self.hand_pose_pub.publish(pose_msg)

                                self._publish_marker(world_pt)
                                self.last_publish_time = now

                cv2.circle(frame, (cx, cy), 5, (0, 255, 0), 2)
                cv2.putText(frame, f'{depth[cy, cx] / 1000.0:.2f}m',
                            (cx, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Annotated image
            if now - self.last_annotated_time >= self.annotated_interval:
                ros_img = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                ros_img.header.stamp = self.get_clock().now().to_msg()
                ros_img.header.frame_id = self.camera_frame
                self.annotated_pub.publish(ros_img)
                self.last_annotated_time = now

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def _publish_marker(self, position):
        """Publish RViz sphere marker at hand position."""
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = HandTrackerNode()

    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = Thread(target=executor.spin)
    spin_thread.start()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        spin_thread.join()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
