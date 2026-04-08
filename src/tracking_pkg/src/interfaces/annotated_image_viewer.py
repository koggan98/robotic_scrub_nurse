#!/usr/bin/env python3

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class AnnotatedImageViewer(Node):
    def __init__(self) -> None:
        super().__init__("annotated_image_viewer")

        self.topic_name = str(self.declare_parameter("topic_name", "/annotated_hand_image").value)
        self.window_name = str(self.declare_parameter("window_name", "Annotated Hand Image").value)

        self.bridge = CvBridge()
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        self.create_subscription(Image, self.topic_name, self._image_callback, 10)
        self.get_logger().info(f"Annotated image viewer listening on {self.topic_name}.")

    def _image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Failed to decode annotated image: {exc}")
            return

        cv2.imshow(self.window_name, frame)
        cv2.waitKey(1)

    def destroy_node(self) -> bool:
        cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AnnotatedImageViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
