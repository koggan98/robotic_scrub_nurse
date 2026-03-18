#!/usr/bin/env python3

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener

from tracking_pkg.srv import GetGraspApproachPose

from grasp_orientation_utils import (
    compute_gripper_quaternion_from_rotation_matrix,
    quaternion_to_rotation_matrix,
)


class GraspApproachPoseService(Node):
    def __init__(self):
        super().__init__("grasp_approach_pose_service")

        self.reference_frame = (
            self.declare_parameter("reference_frame", "world").get_parameter_value().string_value
        )
        self.default_target_frame = (
            self.declare_parameter("default_target_frame", "tool_holder_frame")
            .get_parameter_value()
            .string_value
        )
        self.lookup_timeout_seconds = (
            self.declare_parameter("lookup_timeout_seconds", 0.2)
            .get_parameter_value()
            .double_value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.service = self.create_service(
            GetGraspApproachPose,
            "/get_grasp_approach_pose",
            self.handle_get_grasp_approach_pose,
        )

        self.get_logger().info(
            "Grasp approach pose service ready on /get_grasp_approach_pose "
            f"with reference_frame='{self.reference_frame}' and "
            f"default_target_frame='{self.default_target_frame}'."
        )

    def handle_get_grasp_approach_pose(self, request, response):
        target_frame = request.target_frame.strip() or self.default_target_frame
        response.approach_pose = PoseStamped()
        response.approach_pose.header.frame_id = self.reference_frame

        if not target_frame:
            response.success = False
            response.message = "No target frame was provided and default_target_frame is empty."
            self.get_logger().warn(f"Action rejected: {response.message}")
            return response

        self.get_logger().info(
            f"Action accepted: calculating grasp approach pose for target frame '{target_frame}'."
        )

        try:
            transform = self.tf_buffer.lookup_transform(
                self.reference_frame,
                target_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.lookup_timeout_seconds),
            )

            rotation_matrix = quaternion_to_rotation_matrix(
                [
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w,
                ]
            )
            quaternion_xyzw, warning = compute_gripper_quaternion_from_rotation_matrix(rotation_matrix)
        except (TransformException, ValueError) as exc:
            response.success = False
            response.message = (
                f"Failed to compute grasp approach pose for target frame '{target_frame}': {exc}"
            )
            self.get_logger().warn(f"Action rejected: {response.message}")
            return response

        pose = response.approach_pose
        pose.header.stamp = transform.header.stamp
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation.x = float(quaternion_xyzw[0])
        pose.pose.orientation.y = float(quaternion_xyzw[1])
        pose.pose.orientation.z = float(quaternion_xyzw[2])
        pose.pose.orientation.w = float(quaternion_xyzw[3])

        if warning:
            response.message = (
                f"Computed grasp approach pose for '{target_frame}' with warning: {warning}"
            )
            self.get_logger().warn(response.message)
        else:
            response.message = (
                f"Computed grasp approach pose for '{target_frame}' in '{self.reference_frame}'."
            )
            self.get_logger().info(
                "Computed grasp approach pose: "
                f"x={pose.pose.position.x:.4f}, y={pose.pose.position.y:.4f}, "
                f"z={pose.pose.position.z:.4f}"
            )

        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GraspApproachPoseService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
