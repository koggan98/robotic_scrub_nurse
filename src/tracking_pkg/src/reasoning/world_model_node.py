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

It also OWNS THE TOOL REGISTRY — the inventory of the operation. It belongs here
because this node already sees both trays, the gripper state and the executor's
events, and it already builds the JSON the LLM reads.

  /register_inventory  (std_srvs/Trigger)   -> freeze the tray as this OP's inventory
  /get_tool_home       (GetToolHome)        -> where a tool belongs (for the executor)
  /count_instruments   (CountInstruments)   -> the instrument count

Note the tracker ids (tool_3, reclaim_1) are NOT the registry's identity: they are
evicted after a few seconds of occlusion and re-minted fresh. The registry keys on
its own slots and re-binds track ids on every observation. See tool_registry.py.
"""

import json
import os
from threading import Lock
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from tracking_msgs.msg import (
    GraspCandidateArray,
    HandState,
    SystemState,
    ToolEvent,
)
from tracking_msgs.srv import (
    CountInstruments,
    GetToolCandidates,
    GetToolHome,
    GetWorldModel,
    GetWorldState,
)
from tool_registry import (
    INSTRUMENT_TRAY,
    RECLAIM_TRAY,
    ToolRegistry,
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
        # ── Registry ──
        # The share dir is read-only after install, so the operation's inventory
        # lives under the user's home. Without persistence, a crash of this node
        # mid-operation would take the whole instrument count with it.
        self.declare_parameter(
            'registry_path',
            os.path.join(os.path.expanduser('~'), '.ros', 'rsn_tool_registry.json'))
        # Two same-class tools closer than this cannot be told apart: the tracker
        # (5 cm threshold, greedy) can swap their ids between frames. Registration
        # refuses rather than bind slots that will silently jump.
        self.declare_parameter('min_same_class_gap_m', 0.08)
        self.declare_parameter('home_match_radius_m', 0.12)
        # At count time, a registered tool not seen on its tray within this many
        # seconds is treated as gone (taken away / possibly in the patient).
        # Long enough to survive a brief occlusion, short enough to notice a tool
        # that was actually removed. Only consulted by /count_instruments.
        self.declare_parameter('count_freshness_sec', 2.0)

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
        self.count_freshness_sec = float(
            self.get_parameter('count_freshness_sec').value)

        self._tracker = ToolTracker(threshold, max_age, id_prefix='tool')
        self._reclaim_tracker = ToolTracker(
            threshold, reclaim_max_age, id_prefix='reclaim'
        )
        self._lock = Lock()

        self._registry_path = self.get_parameter('registry_path').value
        self._registry = ToolRegistry(
            min_same_class_gap_m=float(
                self.get_parameter('min_same_class_gap_m').value),
            home_match_radius_m=float(
                self.get_parameter('home_match_radius_m').value),
        )
        self._restore_registry()

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
        # When the instrument-tray perception last delivered a frame. The count
        # reconcile only trusts "not seen = gone" while this stream is live; a
        # dead camera would otherwise make every tool look missing.
        self._last_cand_msg_sec = 0.0

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
        # The transitions perception cannot see (the surgeon taking the tool out of
        # the jaws, above all). See ToolEvent.msg.
        self.create_subscription(
            ToolEvent, '/tool_event', self._tool_event_cb, 10
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
        self.create_service(
            Trigger, '/register_inventory', self._register_inventory_cb
        )
        self.create_service(
            GetToolHome, '/get_tool_home', self._get_tool_home_cb
        )
        self.create_service(
            CountInstruments, '/count_instruments', self._count_instruments_cb
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
            self._last_cand_msg_sec = now_s
            self._registry.observe(
                INSTRUMENT_TRAY, self._to_observations(cands), now_s)

    def _reclaim_cand_cb(self, msg):
        now_s = self.get_clock().now().nanoseconds * 1e-9
        cands = list(msg.candidates)
        cands = self._reclaim_tracker.update(cands, now_s)
        with self._lock:
            self._latest_reclaim_candidates = cands
            self._registry.observe(
                RECLAIM_TRAY, self._to_observations(cands), now_s)

    @staticmethod
    def _to_observations(candidates):
        """GraspCandidates -> the plain dicts the registry speaks.

        Note it is the TOOL pose that goes in (handle centre + long axis), not the
        grasp pose — those differ per tray, and only the tool pose is invariant.
        """
        return [
            {
                'tool_class': c.tool_class,
                'track_id': c.tool_id,
                'handle_xy': (c.handle_center.x, c.handle_center.y),
                'end_dir': (c.functional_end_dir.x, c.functional_end_dir.y),
                'plane_z': c.grasp_pose.pose.position.z,
            }
            for c in candidates
        ]

    def _tool_event_cb(self, msg):
        now_s = self.get_clock().now().nanoseconds * 1e-9
        with self._lock:
            slot = self._registry.on_event(
                msg.event, msg.track_id, msg.tool_class, msg.from_location, now_s)
        if slot is None:
            return
        self.get_logger().info(
            f'ToolEvent {msg.event}: {slot.slot_id} -> {slot.state}')
        self._persist_registry()

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
            'grasp_strategy': c.grasp_strategy,
            'grasp_z_offset_m': float(c.grasp_z_offset),
            'over_opening': bool(c.over_opening),
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

        with self._lock:
            inventory = self._registry.snapshot()

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
            # The operation's inventory: every registered instrument and its state,
            # including the ones that are nowhere visible right now.
            'inventory': inventory,
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

    # ── Registry services ──────────────────────────────────────────

    def _register_inventory_cb(self, request, response):
        """Freeze whatever is on the instrument tray as this operation's inventory.

        Deliberately does NOT check against an expected list — four tools is as
        valid a simulated operation as eleven. It DOES report back exactly what it
        found, so the LLM can read it out and a human notices a tool that was
        occluded. That read-back is the only safeguard, and it is precisely how the
        count is done in a real operating room.
        """
        del request
        now_s = self.get_clock().now().nanoseconds * 1e-9
        with self._lock:
            cands = list(self._latest_candidates)
            ok, found, message = self._registry.register(
                self._to_observations(cands), now_s)

        if not ok:
            self.get_logger().warn(f'register_inventory refused: {message}')
            response.success = False
            response.message = message
            return response

        self._persist_registry()
        detail = ', '.join(f'{n}x {cls}' for cls, n in sorted(found.items()))
        self.get_logger().info(f'Inventory registered: {detail}')
        response.success = True
        response.message = json.dumps({
            'total': sum(found.values()),
            'found': found,
            'message': message,
        })
        return response

    def _get_tool_home_cb(self, request, response):
        with self._lock:
            slot = self._registry.home_of(request.tool_class, request.track_id)
        if slot is None:
            response.success = False
            response.message = (
                f'no_home_for_class: {request.tool_class}'
                if self._registry.is_registered
                else 'no inventory registered — run register_inventory first')
            return response

        response.success = True
        response.message = 'ok'
        response.slot_id = slot.slot_id
        response.home_handle_center.x = float(slot.home_xy[0])
        response.home_handle_center.y = float(slot.home_xy[1])
        response.home_handle_center.z = float(slot.home_plane_z)
        response.home_functional_end_dir.x = float(slot.home_dir[0])
        response.home_functional_end_dir.y = float(slot.home_dir[1])
        response.home_functional_end_dir.z = 0.0
        response.home_plane_z = float(slot.home_plane_z)
        return response

    def _count_instruments_cb(self, request, response):
        del request
        now_s = self.get_clock().now().nanoseconds * 1e-9
        with self._lock:
            # Re-check the cameras before computing the difference: a tool
            # believed at home but no longer seen there was taken away. Only
            # trust "not seen = gone" while perception is actually live —
            # otherwise a stalled camera would flag every tool as missing.
            perception_live = (
                (now_s - self._last_cand_msg_sec) <= self.count_freshness_sec)
            if perception_live:
                demoted = self._registry.reconcile(
                    self.count_freshness_sec, now_s)
                if demoted:
                    self.get_logger().info(
                        f'Count reconcile: {len(demoted)} not seen on a tray '
                        f'-> UNKNOWN: {demoted}')
            else:
                self.get_logger().warn(
                    'Count: instrument perception is stale — counting on the '
                    'last known states without a fresh re-check.')
            c = self._registry.count()
        if perception_live:
            self._persist_registry()
        response.registered = c['registered']
        response.all_accounted_for = c['all_accounted_for']
        response.expected = c['expected']
        response.at_home = c['at_home']
        response.in_gripper = c['in_gripper']
        response.on_reclaim = c['on_reclaim']
        response.in_use = c['in_use']
        response.unknown = c['unknown']
        response.report_json = json.dumps(c)
        if c['unaccounted_for']:
            self.get_logger().warn(
                f'Instrument count: {c["at_home"]}/{c["expected"]} at home. '
                f'NOT ACCOUNTED FOR: {c["unaccounted_for"]}')
        return response

    # ── Registry persistence ───────────────────────────────────────

    def _restore_registry(self):
        if not os.path.exists(self._registry_path):
            return
        try:
            n = self._registry.load(self._registry_path)
            self.get_logger().info(
                f'Restored an inventory of {n} instruments from '
                f'{self._registry_path}. Run register_inventory to start a new '
                f'operation.')
        except Exception as e:
            self.get_logger().warn(
                f'Could not restore the registry from {self._registry_path}: {e}')

    def _persist_registry(self):
        try:
            self._registry.save(self._registry_path)
        except Exception as e:
            # Never let a failing disk take the running system down — but say so,
            # because the inventory would not survive a restart.
            self.get_logger().warn(f'Could not persist the registry: {e}')


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
