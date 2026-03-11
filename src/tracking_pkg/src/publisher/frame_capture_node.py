#!/usr/bin/env python3

from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class FrameCaptureNode(Node):
    def __init__(self) -> None:
        super().__init__("frame_capture_node")

        session_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_output_dir = Path.home() / "frame_captures" / session_stamp

        self.topic_name = str(self.declare_parameter("topic_name", "/color_image").value)
        self.window_name = str(self.declare_parameter("window_name", "Frame Capture").value)
        self.output_dir = Path(
            str(self.declare_parameter("output_dir", str(default_output_dir)).value)
        ).expanduser()
        self.save_every_n_frames = max(
            1, int(self.declare_parameter("save_every_n_frames", 30).value)
        )

        self.bridge = CvBridge()
        self.latest_frame = None
        self.is_recording = False
        self.frame_counter = 0
        self.recording_start_count = 0
        self.recording_frame_counter = 0
        self.waiting_logged = False
        self.shutdown_requested = False

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        self.create_subscription(Image, self.topic_name, self._image_callback, 10)
        self.ui_timer = self.create_timer(1.0 / 30.0, self._ui_loop)

        self.get_logger().info(
            f"Frame capture node listening on {self.topic_name}. Press 'r' to start/stop recording and 'q' to quit."
        )
        self.get_logger().info(
            f"Frames will be written to {self.output_dir}. Saving every {self.save_every_n_frames} frame(s)."
        )

    def _image_callback(self, msg: Image) -> None:
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Failed to decode image from {self.topic_name}: {exc}")
            return

        self.waiting_logged = False

        if self.is_recording:
            self.recording_frame_counter += 1
            if self.recording_frame_counter % self.save_every_n_frames == 0:
                self._save_frame(self.latest_frame)

    def _ui_loop(self) -> None:
        if self.latest_frame is None:
            if not self.waiting_logged:
                self.get_logger().warn(f"Waiting for images on {self.topic_name}.")
                self.waiting_logged = True
            frame_to_show = self._build_waiting_frame()
        else:
            frame_to_show = self.latest_frame.copy()
            if self.is_recording:
                cv2.putText(
                    frame_to_show,
                    "REC",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    frame_to_show,
                    f"Saved: {self.frame_counter}",
                    (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )

        cv2.imshow(self.window_name, frame_to_show)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("r"):
            self._toggle_recording()
        elif key == ord("q"):
            self.get_logger().info("Quit requested from OpenCV window.")
            self.shutdown_requested = True

    def _build_waiting_frame(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(
            frame,
            f"Waiting for {self.topic_name}",
            (60, 220),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            "Press 'r' to record, 'q' to quit",
            (60, 270),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return frame

    def _toggle_recording(self) -> None:
        self.is_recording = not self.is_recording
        self.output_dir.mkdir(parents=True, exist_ok=True)

        if self.is_recording:
            self.recording_start_count = self.frame_counter
            self.recording_frame_counter = 0
            self.get_logger().info(
                f"Recording started. Saving every {self.save_every_n_frames} frame(s) to {self.output_dir}."
            )
            return

        saved_this_run = self.frame_counter - self.recording_start_count
        self.get_logger().info(
            f"Recording stopped. Saved {saved_this_run} frame(s) in this run, {self.frame_counter} total to {self.output_dir}."
        )

    def _save_frame(self, frame) -> None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        file_path = self.output_dir / f"frame_{timestamp}_{self.frame_counter:06d}.png"

        if not cv2.imwrite(str(file_path), frame):
            self.get_logger().error(f"Failed to write frame to {file_path}.")
            return

        self.frame_counter += 1

    def destroy_node(self) -> bool:
        cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrameCaptureNode()

    try:
        while rclpy.ok() and not node.shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
