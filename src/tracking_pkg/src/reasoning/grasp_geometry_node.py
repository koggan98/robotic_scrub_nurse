#!/usr/bin/env python3
"""
Grasp Geometry Node
===================
Subscribes to /detected_tools_obb (ToolDetectionArray from tool_detection_node)
and derives per-tool robot-relevant geometry features:
  - tool long axis (world XY plane) from body_center_3d - handle_center_3d
  - class-specific grasp point near the detected handle
  - functional_end_direction and handle_axis (world frame)
  - grasp rotation around world Z

Publishes GraspCandidateArray on /tool_grasp_candidates. No semantics are
applied here — `handover_rule` stays empty and gets filled later by
tool_semantics_node (Phase 1 Part 3).
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

from tracking_msgs.msg import (
    GraspCandidate,
    GraspCandidateArray,
    ToolDetectionArray,
)
from grasp_point_utils import (
    class_specific_grasp_distance_m,
    grasp_rule_for_class,
    handle_inner_projection,
)


class GraspGeometryNode(Node):
    def __init__(self):
        super().__init__('grasp_geometry_node')

        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('grasp_offset_m', 0.035)
        self.declare_parameter('detections_topic', '/detected_tools_obb')
        self.declare_parameter('candidates_topic', '/tool_grasp_candidates')

        self.world_frame = self.get_parameter('world_frame').value
        self.grasp_offset_m = float(self.get_parameter('grasp_offset_m').value)
        detections_topic = self.get_parameter('detections_topic').value
        candidates_topic = self.get_parameter('candidates_topic').value

        self.create_subscription(
            ToolDetectionArray, detections_topic, self._on_detections, 10
        )
        self.candidates_pub = self.create_publisher(
            GraspCandidateArray, candidates_topic, 10
        )

        self.get_logger().info(
            f'GraspGeometryNode ready ({detections_topic} -> {candidates_topic}, '
            f'grasp_offset={self.grasp_offset_m:.3f} m)'
        )

    def _on_detections(self, msg):
        out = GraspCandidateArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.world_frame

        for det in msg.detections:
            if not det.has_3d:
                continue
            cand = self._build_candidate(det, msg.header.stamp)
            if cand is not None:
                out.candidates.append(cand)

        self.candidates_pub.publish(out)

    def _build_candidate(self, det, stamp):
        body_c = np.array(
            [det.body_center_3d.x, det.body_center_3d.y, det.body_center_3d.z],
            dtype=float,
        )
        handle_c = np.array(
            [det.handle_center_3d.x, det.handle_center_3d.y, det.handle_center_3d.z],
            dtype=float,
        )

        delta_xy = body_c - handle_c
        delta_xy[2] = 0.0
        norm = float(np.linalg.norm(delta_xy))
        if norm < 1e-6:
            self.get_logger().warn(
                f'{det.tool_id}: body/handle centers coincide in XY, skipping'
            )
            return None

        functional_end_dir = delta_xy / norm
        handle_dir = -functional_end_dir

        grasp_distance_m, _ = self._grasp_distance_from_handle(det, norm)
        grasp_point = handle_c + grasp_distance_m * functional_end_dir

        grasp_rot_z = math.atan2(functional_end_dir[1], functional_end_dir[0])
        quat = R.from_euler('z', grasp_rot_z).as_quat()  # x, y, z, w

        cand = GraspCandidate()
        cand.header.stamp = stamp
        cand.header.frame_id = self.world_frame
        cand.tool_id = det.tool_id
        cand.tool_class = det.tool_class
        cand.location = det.location
        cand.grasp_confidence = float(det.confidence)
        cand.handover_rule = ''

        cand.grasp_pose.header.stamp = stamp
        cand.grasp_pose.header.frame_id = self.world_frame
        cand.grasp_pose.pose.position.x = float(grasp_point[0])
        cand.grasp_pose.pose.position.y = float(grasp_point[1])
        cand.grasp_pose.pose.position.z = float(grasp_point[2])
        cand.grasp_pose.pose.orientation.x = float(quat[0])
        cand.grasp_pose.pose.orientation.y = float(quat[1])
        cand.grasp_pose.pose.orientation.z = float(quat[2])
        cand.grasp_pose.pose.orientation.w = float(quat[3])

        cand.approach_direction = Vector3(x=0.0, y=0.0, z=-1.0)
        cand.functional_end_dir = Vector3(
            x=float(functional_end_dir[0]),
            y=float(functional_end_dir[1]),
            z=0.0,
        )
        cand.handle_axis = Vector3(
            x=float(handle_dir[0]),
            y=float(handle_dir[1]),
            z=0.0,
        )

        cand.handover_pose.header.frame_id = self.world_frame
        cand.handover_pose.pose.orientation.w = 1.0

        return cand

    def _grasp_distance_from_handle(self, det, center_dist_m):
        """Class-specific distance from handle center toward tool center."""
        if grasp_rule_for_class(det.tool_class) == 'fallback':
            return self.grasp_offset_m, 'fallback'

        body_px = np.array(
            [det.body_obb.center_x, det.body_obb.center_y],
            dtype=float,
        )
        handle_px = np.array(
            [det.handle_obb.center_x, det.handle_obb.center_y],
            dtype=float,
        )
        center_delta_px = body_px - handle_px
        center_delta_px_norm = float(np.linalg.norm(center_delta_px))
        if center_delta_px_norm < 1e-6 or center_dist_m <= 1e-9:
            return self.grasp_offset_m, 'fallback_no_pixel_scale'

        toward_center_px = center_delta_px / center_delta_px_norm
        handle_corners_px = np.array(
            [[p.x, p.y] for p in det.handle_obb.corners],
            dtype=float,
        )
        inner_edge_px = handle_inner_projection(
            handle_px, handle_corners_px, toward_center_px,
        )
        meters_per_px = center_dist_m / center_delta_px_norm
        inner_edge_m = inner_edge_px * meters_per_px
        return class_specific_grasp_distance_m(
            det.tool_class, inner_edge_m, self.grasp_offset_m,
        )


def main(args=None):
    rclpy.init(args=args)
    node = GraspGeometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
