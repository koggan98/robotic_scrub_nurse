#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive


class TrayCameraVolumePublisher(Node):
    def __init__(self):
        super().__init__("tray_camera_volume_publisher")

        self.frame_id = (
            self.declare_parameter("frame_id", "tray_camera_color_optical_frame")
            .get_parameter_value()
            .string_value
        )
        self.collision_topic = (
            self.declare_parameter("collision_topic", "/collision_object")
            .get_parameter_value()
            .string_value
        )
        self.object_id = (
            self.declare_parameter("object_id", "tray_camera_volume")
            .get_parameter_value()
            .string_value
        )
        self.width_m = (
            self.declare_parameter("width_m", 0.05)
            .get_parameter_value()
            .double_value
        )
        self.height_m = (
            self.declare_parameter("height_m", 0.05)
            .get_parameter_value()
            .double_value
        )
        self.length_m = (
            self.declare_parameter("length_m", 0.40)
            .get_parameter_value()
            .double_value
        )
        self.publish_hz = (
            self.declare_parameter("publish_hz", 2.0)
            .get_parameter_value()
            .double_value
        )

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(
            CollisionObject, self.collision_topic, qos
        )
        period = 1.0 / self.publish_hz if self.publish_hz > 0.0 else 0.5
        self.timer = self.create_timer(period, self.publish_collision_object)
        self.get_logger().info(
            "Publishing tray camera volume collision object "
            f"'{self.object_id}' on {self.collision_topic} in frame "
            f"'{self.frame_id}' ({self.width_m:.3f} x {self.height_m:.3f} x "
            f"{self.length_m:.3f} m)."
        )
        self.publish_collision_object()

    def publish_collision_object(self):
        collision_object = CollisionObject()
        collision_object.header.frame_id = self.frame_id
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = self.object_id
        collision_object.operation = CollisionObject.ADD

        # The RealSense optical +Z axis points away from the camera. In the
        # current setup that is downward toward the tray, so the prism is
        # centered halfway down the camera optical axis.
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = self.length_m / 2.0
        pose.orientation.w = 1.0

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [self.width_m, self.height_m, self.length_m]

        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)

        self.publisher.publish(collision_object)


def main(args=None):
    rclpy.init(args=args)
    node = TrayCameraVolumePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
