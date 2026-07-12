#!/usr/bin/env python3

import time

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive


class InstrumentTrayCollisionPublisher(Node):
    def __init__(self):
        super().__init__("instrument_tray_collision_publisher")

        self.optical_frame = (
            self.declare_parameter("frame_id", "tray_camera_color_optical_frame")
            .get_parameter_value()
            .string_value
        )
        self.world_frame = (
            self.declare_parameter("world_frame", "world")
            .get_parameter_value()
            .string_value
        )
        self.collision_topic = (
            self.declare_parameter("collision_topic", "/collision_object")
            .get_parameter_value()
            .string_value
        )
        self.frame_box_id = (
            self.declare_parameter("frame_box_id", "instrument_tray_frame_box")
            .get_parameter_value()
            .string_value
        )
        self.post_id = (
            self.declare_parameter("post_id", "instrument_tray_vertical_post")
            .get_parameter_value()
            .string_value
        )
        self.frame_box_width_m = (
            self.declare_parameter("frame_box_width_m", 0.14)
            .get_parameter_value()
            .double_value
        )
        self.frame_box_depth_m = (
            self.declare_parameter("frame_box_depth_m", 0.04)
            .get_parameter_value()
            .double_value
        )
        self.frame_box_height_m = (
            self.declare_parameter("frame_box_height_m", 0.04)
            .get_parameter_value()
            .double_value
        )
        self.post_thickness_m = (
            self.declare_parameter("post_thickness_m", 0.05)
            .get_parameter_value()
            .double_value
        )
        self.post_length_m = (
            self.declare_parameter("post_length_m", 0.52)
            .get_parameter_value()
            .double_value
        )
        self.behind_offset_y_m = (
            self.declare_parameter("behind_offset_y_m", 0.03)
            .get_parameter_value()
            .double_value
        )
        self.top_z_offset_m = (
            self.declare_parameter("top_z_offset_m", 0.0)
            .get_parameter_value()
            .double_value
        )
        self.horizontal_bar_length_m = (
            self.declare_parameter("horizontal_bar_length_m", 0.1)
            .get_parameter_value()
            .double_value
        )
        self.horizontal_bar_thickness_m = (
            self.declare_parameter("horizontal_bar_thickness_m", 0.04)
            .get_parameter_value()
            .double_value
        )
        self.cross_block_x_length_m = (
            self.declare_parameter("cross_block_x_length_m", 0.8)
            .get_parameter_value()
            .double_value
        )
        self.cross_block_y_length_m = (
            self.declare_parameter("cross_block_y_length_m", 0.16)
            .get_parameter_value()
            .double_value
        )
        self.cross_block_thickness_m = (
            self.declare_parameter("cross_block_thickness_m", 0.04)
            .get_parameter_value()
            .double_value
        )
        # Raises the two horizontal pieces at the foot of the post (the short
        # connecting bar and the wide cross block) without moving the post itself,
        # which hangs off the camera frame and is already correct. Positive = up.
        # Measured against the real stand: 0.03 sat too high.
        self.horizontal_z_offset_m = (
            self.declare_parameter("horizontal_z_offset_m", 0.01)
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

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._last_tf_warn = 0.0

        period = 1.0 / self.publish_hz if self.publish_hz > 0.0 else 0.5
        self.timer = self.create_timer(period, self.publish_collision_objects)

        self.get_logger().info(
            "Publishing instrument tray collision objects: "
            f"frame_box='{self.frame_box_id}' (in {self.optical_frame}), "
            f"vertical_post='{self.post_id}' (in {self.world_frame}, "
            f"{self.post_thickness_m:.3f} x {self.post_thickness_m:.3f} x "
            f"{self.post_length_m:.3f} m, "
            f"+y offset {self.behind_offset_y_m:.3f} m, "
            f"horizontal parts z_offset {self.horizontal_z_offset_m:+.3f} m)."
        )
        self.publish_collision_objects()

    def publish_collision_objects(self):
        self._publish_frame_box()
        self._publish_vertical_post()

    def _publish_frame_box(self):
        msg = CollisionObject()
        msg.header.frame_id = self.optical_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.id = self.frame_box_id
        msg.operation = CollisionObject.ADD

        pose = Pose()
        pose.orientation.w = 1.0

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [
            self.frame_box_width_m,
            self.frame_box_depth_m,
            self.frame_box_height_m,
        ]

        msg.primitives.append(primitive)
        msg.primitive_poses.append(pose)
        self.publisher.publish(msg)

    def _publish_vertical_post(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame, self.optical_frame, Time()
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            now = time.time()
            if now - self._last_tf_warn >= 2.0:
                self.get_logger().warn(
                    f"Waiting for TF {self.world_frame} -> {self.optical_frame} "
                    "before publishing vertical post."
                )
                self._last_tf_warn = now
            return

        cam_x = tf.transform.translation.x
        cam_y = tf.transform.translation.y
        cam_z = tf.transform.translation.z

        msg = CollisionObject()
        msg.header.frame_id = self.world_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.id = self.post_id
        msg.operation = CollisionObject.ADD

        post_bottom_z = cam_z + self.top_z_offset_m - self.post_length_m
        post_center_y = cam_y + self.behind_offset_y_m

        post_pose = Pose()
        post_pose.position.x = cam_x
        post_pose.position.y = post_center_y
        post_pose.position.z = post_bottom_z + (self.post_length_m / 2.0)
        post_pose.orientation.w = 1.0

        post_primitive = SolidPrimitive()
        post_primitive.type = SolidPrimitive.BOX
        post_primitive.dimensions = [
            self.post_thickness_m,
            self.post_thickness_m,
            self.post_length_m,
        ]

        msg.primitives.append(post_primitive)
        msg.primitive_poses.append(post_pose)

        # Both horizontal pieces below share this height.
        bar_center_z = (
            post_bottom_z
            + (self.horizontal_bar_thickness_m / 2.0)
            + self.horizontal_z_offset_m
        )
        bar_end_y = (
            post_center_y
            - (self.post_thickness_m / 2.0)
            - self.horizontal_bar_length_m
        )

        bar_pose = Pose()
        bar_pose.position.x = cam_x
        bar_pose.position.y = bar_end_y + (self.horizontal_bar_length_m / 2.0)
        bar_pose.position.z = bar_center_z
        bar_pose.orientation.w = 1.0

        bar_primitive = SolidPrimitive()
        bar_primitive.type = SolidPrimitive.BOX
        bar_primitive.dimensions = [
            self.horizontal_bar_thickness_m,
            self.horizontal_bar_length_m,
            self.horizontal_bar_thickness_m,
        ]

        msg.primitives.append(bar_primitive)
        msg.primitive_poses.append(bar_pose)

        cross_pose = Pose()
        cross_pose.position.x = cam_x
        cross_pose.position.y = bar_end_y - (self.cross_block_y_length_m / 2.0)
        cross_pose.position.z = bar_center_z
        cross_pose.orientation.w = 1.0

        cross_primitive = SolidPrimitive()
        cross_primitive.type = SolidPrimitive.BOX
        cross_primitive.dimensions = [
            self.cross_block_x_length_m,
            self.cross_block_y_length_m,
            self.cross_block_thickness_m,
        ]

        msg.primitives.append(cross_primitive)
        msg.primitive_poses.append(cross_pose)

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = InstrumentTrayCollisionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
