#!/usr/bin/env python3
"""
Tool Semantics Node
===================
Subscribes to /tool_grasp_candidates, looks up each tool_class in
tool_knowledge_base.yaml, fills the semantic fields of GraspCandidate
(handover_rule, handover_description, grip_strategy, functional_end_label,
display_name, lift_height), and republishes on /enriched_tool_grasp_candidates.

Class resolution order: direct match → variants → synonyms (case-insensitive)
→ default fallback.
"""

import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from tracking_pkg.msg import GraspCandidate, GraspCandidateArray


class ToolSemanticsNode(Node):
    def __init__(self):
        super().__init__('tool_semantics_node')

        self.declare_parameter('knowledge_base_path', '')
        self.declare_parameter('input_topic', '/tool_grasp_candidates')
        self.declare_parameter('output_topic', '/enriched_tool_grasp_candidates')

        kb_path = self.get_parameter('knowledge_base_path').value
        if not kb_path:
            kb_path = os.path.join(
                get_package_share_directory('tracking_pkg'),
                'config', 'tool_knowledge_base.yaml',
            )
        self._tools, self._default = self._load_kb(kb_path)
        self._alias_to_canonical = self._build_alias_index(self._tools)
        self._unknown_classes_logged = set()

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.create_subscription(
            GraspCandidateArray, input_topic, self._on_candidates, 10
        )
        self.publisher = self.create_publisher(
            GraspCandidateArray, output_topic, 10
        )

        self.get_logger().info(
            f'ToolSemanticsNode ready ({input_topic} -> {output_topic}, '
            f'{len(self._tools)} tool classes loaded from {kb_path})'
        )

    def _load_kb(self, path):
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f) or {}
            return data.get('tools', {}) or {}, data.get('default', {}) or {}
        except Exception as e:
            self.get_logger().error(f'Failed to load knowledge base {path}: {e}')
            return {}, {}

    @staticmethod
    def _build_alias_index(tools):
        index = {}
        for canonical, info in tools.items():
            index[canonical] = canonical
            for v in info.get('variants', []) or []:
                index[v] = canonical
            for s in info.get('synonyms', []) or []:
                index[s.lower()] = canonical
        return index

    def _resolve(self, raw_class):
        if not raw_class:
            return '<unknown>', self._default
        if raw_class in self._alias_to_canonical:
            canonical = self._alias_to_canonical[raw_class]
            return canonical, self._tools[canonical]
        lower = raw_class.lower()
        if lower in self._alias_to_canonical:
            canonical = self._alias_to_canonical[lower]
            return canonical, self._tools[canonical]
        if raw_class not in self._unknown_classes_logged:
            self._unknown_classes_logged.add(raw_class)
            self.get_logger().warn(
                f'Unknown tool_class "{raw_class}" — falling back to default semantics'
            )
        return '<unknown>', self._default

    def _on_candidates(self, msg_in):
        msg_out = GraspCandidateArray()
        msg_out.header = msg_in.header
        for cand in msg_in.candidates:
            msg_out.candidates.append(self._enrich(cand))
        self.publisher.publish(msg_out)

    def _enrich(self, cand):
        canonical, info = self._resolve(cand.tool_class)
        cand.handover_rule = info.get('handover_rule', 'neutral')
        cand.handover_description = info.get('handover_description', '')
        cand.grip_strategy = info.get('grip_strategy', 'parallel')
        cand.functional_end_label = info.get('functional_end', '')
        cand.display_name = info.get('display_name', canonical)
        cand.lift_height = float(info.get('lift_height', 0.10))
        return cand


def main(args=None):
    rclpy.init(args=args)
    node = ToolSemanticsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
