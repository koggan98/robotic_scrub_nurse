#!/usr/bin/env python3
"""
World Model Node
================
On-demand aggregator for the LLM scrub nurse architecture.

Subscribes to:
  /enriched_tool_grasp_candidates  (GraspCandidateArray, semantic-enriched)
  /hand_state                      (HandState)
  /system_state_update             (String, state-machine updates)
  /joint_states                    (JointState)

Maintains persistent tool ids across frames via greedy XY position matching
(class-bound). Provides three services:
  /get_world_state     (GetWorldState)   -> SystemState struct (typed ROS)
  /get_world_model     (GetWorldModel)   -> JSON snapshot string (for LLM)
  /get_tool_candidates (GetToolCandidates) -> filtered candidate list
"""

import json
from threading import Lock
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from tracking_msgs.msg import (
    GraspCandidateArray,
    HandState,
    SystemState,
)
from tracking_msgs.srv import (
    GetToolCandidates,
    GetWorldModel,
    GetWorldState,
)


class _Track:
    __slots__ = ('id', 'xy', 'last_stamp', 'tool_class')

    def __init__(self, id_, xy, last_stamp, tool_class):
        self.id = id_
        self.xy = xy
        self.last_stamp = last_stamp
        self.tool_class = tool_class


class ToolTracker:
    """Greedy XY-distance matcher, class-bound. Evicts stale tracks per update.

    One instance per tray. `id_prefix` keeps the two trays' ids in separate
    namespaces (tool_0, tool_1, ... vs reclaim_0, reclaim_1, ...) so a single
    world model can hold both without id collisions.
    """

    def __init__(self, threshold_m, max_age_s, id_prefix='tool'):
        self.threshold = float(threshold_m)
        self.max_age = float(max_age_s)
        self.id_prefix = id_prefix
        self.tracks = []
        self._counter = 0

    def update(self, candidates, now_s):
        self.tracks = [t for t in self.tracks if now_s - t.last_stamp <= self.max_age]

        used = set()
        for cand in candidates:
            cand_xy = np.array([
                cand.grasp_pose.pose.position.x,
                cand.grasp_pose.pose.position.y,
            ])

            best_i, best_d = None, self.threshold
            for i, t in enumerate(self.tracks):
                if i in used or t.tool_class != cand.tool_class:
                    continue
                d = float(np.linalg.norm(cand_xy - t.xy))
                if d < best_d:
                    best_i, best_d = i, d

            if best_i is not None:
                self.tracks[best_i].xy = cand_xy
                self.tracks[best_i].last_stamp = now_s
                cand.tool_id = self.tracks[best_i].id
                used.add(best_i)
            else:
                new_id = f'{self.id_prefix}_{self._counter}'
                self._counter += 1
                self.tracks.append(
                    _Track(new_id, cand_xy, now_s, cand.tool_class)
                )
                cand.tool_id = new_id

        return candidates


class WorldModelNode(Node):
    def __init__(self):
        super().__init__('world_model_node')

        self.declare_parameter('track_distance_threshold_m', 0.05)
        self.declare_parameter('track_max_age_sec', 3.0)
        self.declare_parameter('candidates_topic', '/enriched_tool_grasp_candidates')
        # Reclaim tray. Empty = disabled, so a launch that does not set it keeps
        # the instrument-tray-only behaviour unchanged.
        self.declare_parameter('reclaim_candidates_topic', '')
        # The reclaim tray changes slowly and the surgeon's hands occlude it often;
        # a 3 s eviction would drop tools on every reach-over. Hence its own age.
        self.declare_parameter('reclaim_track_max_age_sec', 10.0)
        self.declare_parameter('hand_state_topic', '/hand_state')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('hand_confidence_threshold', 0.3)

        threshold = float(self.get_parameter('track_distance_threshold_m').value)
        max_age = float(self.get_parameter('track_max_age_sec').value)
        candidates_topic = self.get_parameter('candidates_topic').value
        reclaim_topic = (self.get_parameter('reclaim_candidates_topic').value or '').strip()
        reclaim_max_age = float(self.get_parameter('reclaim_track_max_age_sec').value)
        hand_state_topic = self.get_parameter('hand_state_topic').value
        self.world_frame = self.get_parameter('world_frame').value
        self.hand_confidence_threshold = float(
            self.get_parameter('hand_confidence_threshold').value
        )

        self._tracker = ToolTracker(threshold, max_age, id_prefix='tool')
        self._reclaim_tracker = ToolTracker(
            threshold, reclaim_max_age, id_prefix='reclaim'
        )
        self._lock = Lock()

        self._latest_candidates = []
        self._latest_reclaim_candidates = []
        self._hand_state: Optional[HandState] = None
        self._hand_available = False
        self._state = 'IDLE'
        self._active_tool_id = ''
        self._active_tool_class = ''
        self._tcp_pose = PoseStamped()
        self._tcp_pose.header.frame_id = self.world_frame
        self._joint_positions = [0.0] * 6
        self._robot_ready = True
        self._last_gesture = ''
        self._last_gesture_sec = 0.0
        # Ground truth from the gripper (robotiq object register). None = unknown
        # (no message yet), True = a tool is physically held, False = empty.
        self._gripper_holds_tool = None

        self.create_subscription(
            GraspCandidateArray, candidates_topic, self._cand_cb, 10
        )
        if reclaim_topic:
            self.create_subscription(
                GraspCandidateArray, reclaim_topic, self._reclaim_cand_cb, 10
            )
        self.create_subscription(
            HandState, hand_state_topic, self._hand_cb, 10
        )
        self.create_subscription(
            String, '/system_state_update', self._state_cb, 10
        )
        self.create_subscription(
            JointState, '/joint_states', self._joints_cb, 10
        )
        self.create_subscription(
            String, '/hand_gesture', self._gesture_cb, 10
        )
        self.create_subscription(
            Bool, '/tool_grasped', self._tool_grasped_cb, 10
        )

        self.create_service(
            GetWorldState, '/get_world_state', self._get_world_state_cb
        )
        self.create_service(
            GetToolCandidates, '/get_tool_candidates', self._get_tool_candidates_cb
        )
        self.create_service(
            GetWorldModel, '/get_world_model', self._get_world_model_cb
        )

        self.get_logger().info(
            f'WorldModelNode ready (track_threshold={threshold:.3f} m, '
            f'max_age={max_age:.1f} s, candidates={candidates_topic}, '
            f'reclaim={reclaim_topic or "<disabled>"} '
            f'(max_age={reclaim_max_age:.1f} s), '
            f'hand_state={hand_state_topic})'
        )

    # ── Push callbacks ─────────────────────────────────────────────

    def _cand_cb(self, msg):
        now_s = self.get_clock().now().nanoseconds * 1e-9
        cands = list(msg.candidates)
        cands = self._tracker.update(cands, now_s)
        with self._lock:
            self._latest_candidates = cands

    def _reclaim_cand_cb(self, msg):
        now_s = self.get_clock().now().nanoseconds * 1e-9
        cands = list(msg.candidates)
        cands = self._reclaim_tracker.update(cands, now_s)
        with self._lock:
            self._latest_reclaim_candidates = cands

    def _hand_cb(self, msg):
        with self._lock:
            self._hand_state = msg
            self._hand_available = (
                msg.is_tracked and msg.confidence > self.hand_confidence_threshold
            )

    def _state_cb(self, msg):
        parts = msg.data.split(':')
        with self._lock:
            self._state = parts[0] if parts else 'IDLE'
            self._active_tool_id = parts[1] if len(parts) > 1 else ''
            self._active_tool_class = parts[2] if len(parts) > 2 else ''
        self.get_logger().info(
            f'State -> {self._state} (tool: {self._active_tool_id})'
        )

    def _tool_grasped_cb(self, msg):
        with self._lock:
            self._gripper_holds_tool = bool(msg.data)

    def _joints_cb(self, msg):
        if len(msg.position) >= 6:
            with self._lock:
                self._joint_positions = list(msg.position[:6])

    def _gesture_cb(self, msg):
        now_s = self.get_clock().now().nanoseconds * 1e-9
        with self._lock:
            self._last_gesture = msg.data
            self._last_gesture_sec = now_s
        self.get_logger().info(f'Gesture received: {msg.data}')

    # ── Service callbacks ──────────────────────────────────────────

    def _snapshot(self):
        with self._lock:
            return (
                list(self._latest_candidates),
                list(self._latest_reclaim_candidates),
                self._hand_state,
                self._hand_available,
                self._state,
                self._active_tool_id,
                self._active_tool_class,
                list(self._joint_positions),
                self._robot_ready,
                self._last_gesture,
                self._last_gesture_sec,
                self._gripper_holds_tool,
            )

    def _get_world_state_cb(self, request, response):
        del request
        (cands, reclaim_cands, hand, hand_avail, state, atid, atc, joints,
         robot_ready, _, _, _) = self._snapshot()

        # Both trays go into tool_candidates so the executor can resolve any
        # tool_id it is handed; each candidate carries its own `location`.
        all_cands = cands + reclaim_cands

        s = SystemState()
        s.header.stamp = self.get_clock().now().to_msg()
        s.header.frame_id = self.world_frame
        s.state = state
        s.active_tool_id = atid
        s.active_tool_class = atc
        s.tool_candidates = all_cands
        s.target_hand = hand if hand is not None else HandState()
        s.hand_available = hand_avail
        s.tcp_pose = self._tcp_pose
        s.joint_positions = joints
        s.robot_ready = robot_ready

        response.success = True
        response.message = (
            f'{len(cands)} tray tools, {len(reclaim_cands)} reclaim tools, '
            f'hand_available={hand_avail}'
        )
        response.world_state = s
        return response

    def _get_tool_candidates_cb(self, request, response):
        cands, reclaim_cands, *_ = self._snapshot()
        all_cands = cands + reclaim_cands
        filt = (request.tool_class_filter or '').strip().lower()
        if filt:
            all_cands = [c for c in all_cands if c.tool_class.lower() == filt]
        response.success = True
        response.message = f'{len(all_cands)} candidates'
        response.candidates = all_cands
        return response

    @staticmethod
    def _tool_to_dict(c):
        return {
            'id': c.tool_id,
            'class': c.tool_class,
            'location': c.location,
            'display_name': c.display_name,
            'confidence': float(c.grasp_confidence),
            'pose_frame': c.grasp_pose.header.frame_id,
            'grasp_point': [
                float(c.grasp_pose.pose.position.x),
                float(c.grasp_pose.pose.position.y),
                float(c.grasp_pose.pose.position.z),
            ],
            'grasp_orientation_quat': [
                float(c.grasp_pose.pose.orientation.x),
                float(c.grasp_pose.pose.orientation.y),
                float(c.grasp_pose.pose.orientation.z),
                float(c.grasp_pose.pose.orientation.w),
            ],
            'handle_axis': [
                float(c.handle_axis.x),
                float(c.handle_axis.y),
                float(c.handle_axis.z),
            ],
            'functional_end_axis': [
                float(c.functional_end_dir.x),
                float(c.functional_end_dir.y),
                float(c.functional_end_dir.z),
            ],
            'approach_direction': [
                float(c.approach_direction.x),
                float(c.approach_direction.y),
                float(c.approach_direction.z),
            ],
            'preferred_handover_rule': c.handover_rule,
            'handover_description': c.handover_description,
            'functional_end_label': c.functional_end_label,
            'grip_strategy': c.grip_strategy,
            'lift_height_m': float(c.lift_height),
        }

    def _get_world_model_cb(self, request, response):
        del request
        (cands, reclaim_cands, hand, _, state, atid, atc, joints, robot_ready,
         last_gesture, last_gesture_sec, gripper_holds_tool) = self._snapshot()
        now_s = self.get_clock().now().nanoseconds * 1e-9

        available_tools = [self._tool_to_dict(c) for c in cands]
        reclaim_tools = [self._tool_to_dict(c) for c in reclaim_cands]

        if hand is not None and hand.is_tracked:
            receiver_hand = {
                'detected': True,
                'confidence': float(hand.confidence),
                'pose': [
                    float(hand.hand_pose.pose.position.x),
                    float(hand.hand_pose.pose.position.y),
                    float(hand.hand_pose.pose.position.z),
                ],
                'frame': hand.hand_pose.header.frame_id or self.world_frame,
                'wrist': [
                    float(hand.wrist.x),
                    float(hand.wrist.y),
                    float(hand.wrist.z),
                ],
                'palm_center': [
                    float(hand.palm_center.x),
                    float(hand.palm_center.y),
                    float(hand.palm_center.z),
                ],
            }
        else:
            receiver_hand = {'detected': False}

        gesture_age = (now_s - last_gesture_sec) if last_gesture_sec > 0.0 else None
        gesture_block = {
            'name': last_gesture if (gesture_age is not None and gesture_age < 30.0) else '',
            'age_sec': gesture_age,
            'fresh': gesture_age is not None and gesture_age < 3.0,
        }

        snapshot = {
            'timestamp_sec': now_s,
            'world_frame': self.world_frame,
            'system_state': state,
            'active_tool_id': atid,
            'active_tool_class': atc,
            'gripper_holds_tool': gripper_holds_tool,
            'robot_ready': robot_ready,
            'joint_positions': joints,
            'available_tools': available_tools,
            'reclaim_tools': reclaim_tools,
            'receiver_hand': receiver_hand,
            'last_gesture': gesture_block,
        }

        response.success = True
        response.message = (
            f'{len(available_tools)} tools, {len(reclaim_tools)} reclaim, '
            f'hand_detected={receiver_hand.get("detected", False)}'
        )
        response.world_model_json = json.dumps(snapshot, indent=2)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = WorldModelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
