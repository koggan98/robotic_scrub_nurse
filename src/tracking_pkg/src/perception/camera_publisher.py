#!/usr/bin/env python3

#Done
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np

class RealSensePublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # Publisher für Tiefen-, RGB-Bild und Cam Parameter
        self.depth_pub = self.create_publisher(Image, 'depth_image', 10)
        self.color_pub = self.create_publisher(Image, 'color_image', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera_info', 10)

        # CvBridge für die Konvertierung
        self.bridge = CvBridge()

        # RealSense Kamera initialisieren
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Starten des Streams
        self.pipeline.start(config)

        # Kameraparameter auslesen
        self.profile = self.pipeline.get_active_profile()
        self.rgb_stream = self.profile.get_stream(rs.stream.color).as_video_stream_profile()
        self.depth_stream = self.profile.get_stream(rs.stream.depth).as_video_stream_profile()

        self.rgb_intrinsics = self.rgb_stream.get_intrinsics()
        self.depth_intrinsics = self.depth_stream.get_intrinsics()
        self.extrinsics = self.depth_stream.get_extrinsics_to(self.rgb_stream)

        # Jetzt publish_camera_info() aufrufen
        self.publish_camera_info()

        # alignment initialisieren
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        # Filter initialisieren
        self.decimation_filter = rs.decimation_filter()
        self.decimation_filter.set_option(rs.option.filter_magnitude, 2)

        self.threshold_filter = rs.threshold_filter()
        self.threshold_filter.set_option(rs.option.min_distance, 0.1)
        self.threshold_filter.set_option(rs.option.max_distance, 4.0)

        self.hdr_merge = rs.hdr_merge()
        self.depth_to_disparity = rs.disparity_transform(True)

        self.spatial_filter = rs.spatial_filter()
        self.spatial_filter.set_option(rs.option.filter_magnitude, 2)
        self.spatial_filter.set_option(rs.option.filter_smooth_alpha, 0.5)
        self.spatial_filter.set_option(rs.option.filter_smooth_delta, 20)
        self.spatial_filter.set_option(rs.option.holes_fill, 1)

        self.temporal_filter = rs.temporal_filter()
        self.temporal_filter.set_option(rs.option.filter_smooth_alpha, 0.4)
        self.temporal_filter.set_option(rs.option.filter_smooth_delta, 20)
        self.temporal_filter.set_option(rs.option.holes_fill, 2)

        self.hole_filling_filter = rs.hole_filling_filter()
        self.hole_filling_filter.set_option(rs.option.holes_fill, 2)

        self.disparity_to_depth = rs.disparity_transform(False)

        # Timer für regelmäßiges Veröffentlichen
        self.timer = self.create_timer(0.03, self.publish_images)  # 30 Hz
        self.get_logger().info("Camera Node initialized.")

    def publish_images(self):
        
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)  # Hier passiert das Alignment

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        # Filter anwenden
        if depth_frame:
            depth_frame = self.hdr_merge.process(depth_frame)
            depth_frame = self.threshold_filter.process(depth_frame)
            depth_frame = self.depth_to_disparity.process(depth_frame)
            depth_frame = self.spatial_filter.process(depth_frame)
            depth_frame = self.temporal_filter.process(depth_frame)
            depth_frame = self.disparity_to_depth.process(depth_frame)
            depth_frame = self.hole_filling_filter.process(depth_frame)

            depth_image = np.asanyarray(depth_frame.get_data())
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='16UC1')
            self.depth_pub.publish(depth_msg)

        if color_frame:
            color_image = np.asanyarray(color_frame.get_data())
            color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
            self.color_pub.publish(color_msg)
            self.publish_camera_info()

    def publish_camera_info(self):
        """Publiziert die Kamerainformationen."""
        # Debug-Logs zur Überprüfung der Kameraparameter
        # self.get_logger().info(f"fx: {self.rgb_intrinsics.fx}, fy: {self.rgb_intrinsics.fy}, "f"ppx: {self.rgb_intrinsics.ppx}, ppy: {self.rgb_intrinsics.ppy}")

        # Explizite Konvertierung in float
        camera_info_msg = CameraInfo()
        camera_info_msg.k = [
            float(self.rgb_intrinsics.fx), 0.0, float(self.rgb_intrinsics.ppx),
            0.0, float(self.rgb_intrinsics.fy), float(self.rgb_intrinsics.ppy),
            0.0, 0.0, 1.0
        ]
        self.camera_info_pub.publish(camera_info_msg)


    def on_shutdown(self):
        self.pipeline.stop()

def main(args=None):
    rclpy.init(args=args)
    node = RealSensePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
