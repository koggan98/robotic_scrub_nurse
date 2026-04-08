#!/usr/bin/env python3
"""
World Model Node
================
Central state aggregator that receives push updates from all perception
and execution nodes, and provides snapshot services for the LLM node.

Subscriptions (push updates):
  /grasp_candidates (GraspCandidateArray)
  /hand_state (HandState)
  /system_state_update (String) - state machine transitions from execution
  /joint_states (JointState) - robot joint positions

Services (pull for LLM):
  /get_world_state (GetWorldState) → full SystemState snapshot
  /get_tool_candidates (GetToolCandidates) → filtered tool list
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

# from tracking_pkg.msg import (
#     GraspCandidateArray, HandState, SystemState, GraspCandidate
# )
# from tracking_pkg.srv import GetWorldState, GetToolCandidates


class WorldModelNode(Node):
    def __init__(self):
        super().__init__('world_model_node')

        # Internal state
        self._state = 'IDLE'
        self._active_tool_id = ''
        self._active_tool_class = ''
        self._tool_candidates = []
        self._hand_state = None
        self._hand_available = False
        self._tcp_pose = PoseStamped()
        self._joint_positions = [0.0] * 6
        self._robot_ready = True

        # Subscribers
        # self.create_subscription(
        #     GraspCandidateArray, 'grasp_candidates', self._grasp_cb, 10)
        # self.create_subscription(
        #     HandState, 'hand_state', self._hand_cb, 10)
        self.create_subscription(
            String, 'system_state_update', self._state_cb, 10)
        self.create_subscription(
            JointState, 'joint_states', self._joints_cb, 10)

        # Services
        # self.create_service(
        #     GetWorldState, 'get_world_state', self._get_world_state_cb)
        # self.create_service(
        #     GetToolCandidates, 'get_tool_candidates', self._get_tool_candidates_cb)

        self.get_logger().info('WorldModelNode initialized')

    # ── Push update callbacks ──────────────────────────────────────

    def _grasp_cb(self, msg):
        """Update tool candidates from grasp reasoning."""
        self._tool_candidates = list(msg.candidates)
        self.get_logger().debug(f'Updated {len(self._tool_candidates)} tool candidates')

    def _hand_cb(self, msg):
        """Update hand state from scene perception."""
        self._hand_state = msg
        self._hand_available = msg.is_tracked and msg.confidence > 0.3

    def _state_cb(self, msg):
        """Update system state from execution node."""
        parts = msg.data.split(':')
        self._state = parts[0] if parts else 'IDLE'
        if len(parts) > 1:
            self._active_tool_id = parts[1]
        if len(parts) > 2:
            self._active_tool_class = parts[2]
        self.get_logger().info(f'State → {self._state} (tool: {self._active_tool_id})')

    def _joints_cb(self, msg):
        """Update joint positions from robot driver."""
        if len(msg.position) >= 6:
            self._joint_positions = list(msg.position[:6])

    # ── Service callbacks ──────────────────────────────────────────

    def _get_world_state_cb(self, request, response):
        """Return full world state snapshot."""
        response.success = True
        response.message = 'ok'
        # TODO: populate response.world_state from internal state
        return response

    def _get_tool_candidates_cb(self, request, response):
        """Return tool candidates, optionally filtered by class."""
        filter_class = request.tool_class_filter.strip().lower()
        if filter_class:
            matched = [c for c in self._tool_candidates
                       if c.tool_class.lower() == filter_class]
        else:
            matched = list(self._tool_candidates)

        response.success = True
        response.message = f'{len(matched)} candidates'
        response.candidates = matched
        return response


def main(args=None):
    rclpy.init(args=args)
    node = WorldModelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
