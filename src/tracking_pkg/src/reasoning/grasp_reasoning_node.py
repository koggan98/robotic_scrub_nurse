#!/usr/bin/env python3
"""
Grasp Reasoning Node
====================
Subscribes to tool detections and computes grasp candidates with
semantic axes and handover rules from the tool knowledge base.

Pipeline:
  /tools_detected → compute grasp point (handle centroid)
                   → derive semantic axes (functional_end_dir, handle_axis)
                   → apply handover rules from knowledge base
                   → publish GraspCandidateArray

Publishes:
  /grasp_candidates (tracking_pkg/msg/GraspCandidateArray)

Parameters:
  knowledge_base_path (str): Path to tool_knowledge_base.yaml
"""

import rclpy
from rclpy.node import Node
import yaml
import numpy as np
from ament_index_python.packages import get_package_share_directory
import os

# from tracking_pkg.msg import (
#     ToolDetectionArray, GraspCandidate, GraspCandidateArray
# )


class GraspReasoningNode(Node):
    def __init__(self):
        super().__init__('grasp_reasoning_node')

        self.declare_parameter('knowledge_base_path', '')

        kb_path = self.get_parameter('knowledge_base_path').value
        if not kb_path:
            pkg_share = get_package_share_directory('tracking_pkg')
            kb_path = os.path.join(pkg_share, 'config', 'tool_knowledge_base.yaml')

        self.knowledge_base = self._load_knowledge_base(kb_path)

        # Subscribers
        # self.create_subscription(ToolDetectionArray, 'tools_detected', self._detections_cb, 10)

        # Publisher
        # self.grasp_pub = self.create_publisher(GraspCandidateArray, 'grasp_candidates', 10)

        self.get_logger().info(f'GraspReasoningNode initialized ({len(self.knowledge_base)} tool classes loaded)')

    def _load_knowledge_base(self, path):
        """Load tool knowledge base from YAML."""
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            tools = data.get('tools', {})
            self.default_kb = data.get('default', {})
            self.get_logger().info(f'Loaded knowledge base from {path}')
            return tools
        except Exception as e:
            self.get_logger().error(f'Failed to load knowledge base: {e}')
            self.default_kb = {}
            return {}

    def _get_tool_info(self, tool_class):
        """Look up tool class in knowledge base, with fallback to default."""
        if tool_class in self.knowledge_base:
            return self.knowledge_base[tool_class]
        # Try matching via synonyms
        for cls_name, info in self.knowledge_base.items():
            synonyms = info.get('synonyms', [])
            if tool_class.lower() in [s.lower() for s in synonyms]:
                return info
        return self.default_kb

    def _compute_grasp_from_obbs(self, body_obb, handle_obb):
        """
        Compute grasp point and semantic axes from body and handle OBBs.

        Grasp point: Handle OBB centroid (3D)
        Handle axis: Long axis of handle OBB
        Functional end direction: Vector from handle centroid → body centroid
        Approach direction: Top-down (-Z in world frame)
        """
        # Grasp point = handle centroid
        grasp_point = np.array([
            handle_obb.center_x,
            handle_obb.center_y,
            0.0  # Will be filled from 3D projection
        ])

        # Handle axis = long axis direction of handle OBB
        angle = handle_obb.angle_rad
        if handle_obb.width >= handle_obb.height:
            handle_axis = np.array([np.cos(angle), np.sin(angle), 0.0])
        else:
            handle_axis = np.array([-np.sin(angle), np.cos(angle), 0.0])

        # Functional end direction = body center - handle center
        functional_end_dir = np.array([
            body_obb.center_x - handle_obb.center_x,
            body_obb.center_y - handle_obb.center_y,
            0.0
        ])
        norm = np.linalg.norm(functional_end_dir)
        if norm > 1e-6:
            functional_end_dir /= norm

        return grasp_point, handle_axis, functional_end_dir

    def _detections_cb(self, msg):
        """Process tool detections and publish grasp candidates."""
        # TODO: Implement full pipeline
        # For each detection in msg.detections:
        #   1. Compute grasp point from handle OBB centroid (3D)
        #   2. Derive handle_axis and functional_end_dir
        #   3. Look up handover_rule from knowledge base
        #   4. Construct GraspCandidate message
        #   5. Compute grasp_pose (position + orientation for robot TCP)
        #   6. Compute handover_pose based on handover_rule
        pass


def main(args=None):
    rclpy.init(args=args)
    node = GraspReasoningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
