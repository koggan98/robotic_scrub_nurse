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
  - the grasp strategy: where along the tool to grasp, and how deep

Grasp strategies come from the `grasp_strategies` block of
tool_knowledge_base.yaml, keyed on the RAW model class (forceps_big vs
forceps_short — the `tools:` block collapses those into one canonical entry and
so cannot express the distinction). They are loaded HERE and not in
tool_semantics_node, because that node runs downstream of this one and would
arrive long after the grasp point was computed.

Thin instruments (scissors, needle holders) are so flat that at the tray surface
the jaws only catch their top 2-3 mm, and they slip out. But the trays are ITEM
extrusion frames with an OPEN HOLE in the middle, and those tools are long enough
to be grasped anywhere along the shaft. So for them we SLIDE the grasp point along
the tool's long axis until it lies over the opening, and only there descend below
the tray surface.

⚠ Neither tray's surface is a MoveIt collision object. The opening test in
tray_geometry_utils is the ONLY thing preventing a crash into a profile bar, so a
negative z offset is emitted ONLY when the grasp point and BOTH fingertips are
confirmed inside a measured opening.

Publishes GraspCandidateArray on /tool_grasp_candidates. No semantics are
applied here — `handover_rule` stays empty and gets filled later by
tool_semantics_node.
"""

import math
import os

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Vector3
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

from tracking_msgs.msg import (
    GraspCandidate,
    GraspCandidateArray,
    ToolDetectionArray,
)
from grasp_point_utils import (
    class_specific_grasp_distance_m,
    handle_inner_projection,
)
from tray_geometry_utils import TrayOpenings, gripper_footprint

# Below this pixel distance between the body and handle OBB centres, the
# meters-per-pixel scale (and everything derived from it: the handle's inner edge,
# the tool's length) is numerically junk. Fall back to the legacy flat offset.
_MIN_CENTER_DELTA_PX = 1e-6


class GraspGeometryNode(Node):
    def __init__(self):
        super().__init__('grasp_geometry_node')

        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('grasp_offset_m', 0.035)
        self.declare_parameter('detections_topic', '/detected_tools_obb')
        self.declare_parameter('candidates_topic', '/tool_grasp_candidates')
        self.declare_parameter('knowledge_base_path', '')
        self.declare_parameter('tray_geometry_path', '')
        # Half the gripper's OUTER width across the tool axis, i.e. how far the
        # outer edge of each finger sits from the grasp point while descending.
        # The executor orients the jaws perpendicular to the tool
        # (tool_yaw_offset_rad = pi/2), so these are the two points that would hit
        # a profile bar. Measured: 65 mm outer edge to outer edge -> 32.5 mm.
        self.declare_parameter('finger_half_span_m', 0.0325)
        # Step size when searching along the tool axis for a spot over the opening.
        self.declare_parameter('slide_step_m', 0.002)

        self.world_frame = self.get_parameter('world_frame').value
        self.grasp_offset_m = float(self.get_parameter('grasp_offset_m').value)
        detections_topic = self.get_parameter('detections_topic').value
        candidates_topic = self.get_parameter('candidates_topic').value
        self.finger_half_span_m = float(
            self.get_parameter('finger_half_span_m').value)
        self.slide_step_m = float(self.get_parameter('slide_step_m').value)

        self._strategies = self._load_strategies(
            self.get_parameter('knowledge_base_path').value)
        self._trays = self._load_tray_geometry(
            self.get_parameter('tray_geometry_path').value)

        self.create_subscription(
            ToolDetectionArray, detections_topic, self._on_detections, 10
        )
        self.candidates_pub = self.create_publisher(
            GraspCandidateArray, candidates_topic, 10
        )

        trays_desc = ', '.join(
            f'{name}({len(t.polygons)} opening(s))'
            for name, t in self._trays.items()
        ) or '<none>'
        self.get_logger().info(
            f'GraspGeometryNode ready ({detections_topic} -> {candidates_topic}, '
            f'grasp_offset={self.grasp_offset_m:.3f} m, '
            f'{len(self._strategies)} grasp strategies, trays: {trays_desc}, '
            f'finger_half_span={self.finger_half_span_m:.3f} m)'
        )

    # ── Config ──────────────────────────────────────────────────────

    def _load_strategies(self, path):
        if not path:
            path = os.path.join(
                get_package_share_directory('tracking_pkg'),
                'config', 'tool_knowledge_base.yaml',
            )
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().error(
                f'Failed to load grasp strategies from {path}: {e}. '
                'Falling back to the legacy hard-coded grasp rules.')
            return {}
        strategies = data.get('grasp_strategies', {}) or {}
        if not strategies:
            self.get_logger().warn(
                f'{path} has no grasp_strategies block — falling back to the '
                'legacy hard-coded grasp rules (no per-class z, no sliding).')
        return strategies

    def _load_tray_geometry(self, path):
        if not path:
            path = os.path.join(
                get_package_share_directory('tracking_pkg'),
                'config', 'tray_geometry.yaml',
            )
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f) or {}
        except Exception as e:
            # No geometry = no openings = never descend below the tray surface.
            # That is the safe direction, so warn rather than die.
            self.get_logger().warn(
                f'Failed to load tray geometry from {path}: {e}. '
                'No openings known — all grasps stay at the tray surface.')
            return {}
        return {
            name: TrayOpenings.from_config(cfg)
            for name, cfg in (data.get('trays', {}) or {}).items()
        }

    def _strategy_for(self, tool_class):
        return (self._strategies.get(tool_class)
                or self._strategies.get('default')
                or {})

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

        # Pixel scale, and everything derived from it. Hoisted out of the old
        # _grasp_distance_from_handle, which only computed it on the non-fallback
        # branch — so forceps/retractor/awl never had a tool length.
        inner_edge_m, tool_length_m, scale_ok = self._pixel_derived(det, norm)

        grasp_distance_m, z_offset_m, over_opening, strategy = self._plan_grasp(
            det, handle_c, functional_end_dir, norm,
            inner_edge_m, tool_length_m, scale_ok,
        )
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
        cand.grasp_z_offset = float(z_offset_m)
        cand.over_opening = bool(over_opening)
        cand.grasp_strategy = strategy

        cand.grasp_pose.header.stamp = stamp
        cand.grasp_pose.header.frame_id = self.world_frame
        cand.grasp_pose.pose.position.x = float(grasp_point[0])
        cand.grasp_pose.pose.position.y = float(grasp_point[1])
        cand.grasp_pose.pose.position.z = float(grasp_point[2])
        cand.grasp_pose.pose.orientation.x = float(quat[0])
        cand.grasp_pose.pose.orientation.y = float(quat[1])
        cand.grasp_pose.pose.orientation.z = float(quat[2])
        cand.grasp_pose.pose.orientation.w = float(quat[3])

        # The TOOL's pose, not the grasp pose. grasp_point sits at
        # handle_center + grasp_distance_m * functional_end_dir, and that distance
        # differs per tray (the sliding search uses that tray's opening). Anything
        # that wants to put the tool back where it belongs has to reconstruct the
        # tool pose from these two — replaying the grasp pose would leave it
        # displaced along its own axis. See GraspCandidate.handle_center.
        cand.handle_center = Point(
            x=float(handle_c[0]), y=float(handle_c[1]), z=float(handle_c[2]))

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

    def _pixel_derived(self, det, center_dist_m):
        """Metric quantities recovered from the pixel-space OBBs.

        Returns (inner_edge_m, tool_length_m, scale_ok). `scale_ok` is False when
        the handle sits so close to the body centre that the pixel distance in the
        denominator makes the scale meaningless — then neither the handle's inner
        edge nor the tool's length can be trusted, and the caller must not slide.
        """
        body_px = np.array(
            [det.body_obb.center_x, det.body_obb.center_y], dtype=float)
        handle_px = np.array(
            [det.handle_obb.center_x, det.handle_obb.center_y], dtype=float)

        center_delta_px = body_px - handle_px
        center_delta_px_norm = float(np.linalg.norm(center_delta_px))
        if (center_delta_px_norm < _MIN_CENTER_DELTA_PX
                or center_dist_m <= 1e-9):
            return 0.0, 0.0, False

        meters_per_px = center_dist_m / center_delta_px_norm

        toward_center_px = center_delta_px / center_delta_px_norm
        handle_corners_px = np.array(
            [[p.x, p.y] for p in det.handle_obb.corners], dtype=float)
        inner_edge_m = handle_inner_projection(
            handle_px, handle_corners_px, toward_center_px) * meters_per_px

        # The body OBB's long side is the tool's length. Already on the wire,
        # previously discarded.
        tool_length_m = (
            max(float(det.body_obb.width), float(det.body_obb.height))
            * meters_per_px
        )
        return inner_edge_m, tool_length_m, True

    def _nominal_distance(self, strat, inner_edge_m, scale_ok):
        """Distance from the handle centre along the tool axis, per the strategy.

        Reproduces grasp_point_utils exactly:
          from_handle_inner_edge -> inner edge + distance_m  (old inner_plus)
          from_handle_center     -> distance_m               (old fallback)
        """
        distance_m = float(strat.get('distance_m', self.grasp_offset_m))
        rule = strat.get('distance_rule', 'from_handle_center')
        if rule == 'from_handle_inner_edge':
            if not scale_ok:
                # No usable pixel scale -> no usable inner edge. Degrade the way
                # the old code did: flat offset from the handle centre.
                return self.grasp_offset_m
            return inner_edge_m + distance_m
        return distance_m

    def _plan_grasp(self, det, handle_c, fed, center_dist_m,
                    inner_edge_m, tool_length_m, scale_ok):
        """Pick the grasp distance along the tool axis and the z offset.

        Returns (distance_m, z_offset_m, over_opening, strategy_tag).
        """
        # No strategies loaded at all -> exactly the legacy behaviour.
        if not self._strategies:
            d, _ = class_specific_grasp_distance_m(
                det.tool_class, inner_edge_m if scale_ok else 0.0,
                self.grasp_offset_m,
            )
            return d, 0.0, False, 'legacy'

        strat = self._strategy_for(det.tool_class)
        nominal_d = self._nominal_distance(strat, inner_edge_m, scale_ok)
        shallow_z = float(strat.get('z_offset_m', 0.0))

        if strat.get('placement') != 'sliding':
            return nominal_d, shallow_z, False, 'fixed'

        # ── A sliding tool we cannot safely slide ────────────────────
        # It stays at the tray surface, which is safe but grips poorly. Say WHY,
        # because each reason has a different fix and the tag alone
        # ('sliding_no_opening') does not tell them apart.
        tray = self._trays.get(det.location)
        if not tray:
            return self._no_slide(
                det, nominal_d, shallow_z,
                f'no measured opening for {det.location} — run tray_opening_calib.py')
        if not scale_ok:
            return self._no_slide(
                det, nominal_d, shallow_z,
                'pixel scale unusable (handle too close to the body centre)')

        deep_z = float(strat.get('z_offset_over_opening_m', shallow_z))

        # Search window along the axis, measured from the handle centre.
        d_lo = float(strat.get('slide_min_m', nominal_d))
        # The functional end sits half a tool-length beyond the body centre.
        dist_to_tip = center_dist_m + 0.5 * tool_length_m
        slide_max = float(strat.get('slide_max_m', nominal_d))
        tip_margin = float(strat.get('tip_margin_m', 0.02))
        d_hi = min(slide_max, dist_to_tip - tip_margin)
        if d_hi < d_lo:
            return self._no_slide(
                det, nominal_d, shallow_z,
                f'slide window empty ({d_lo * 1000:.0f}..{d_hi * 1000:.0f} mm) — '
                f'raise slide_max_m or lower tip_margin_m for {det.tool_class}')

        step = max(1e-4, self.slide_step_m)
        feasible = [
            float(d) for d in np.arange(d_lo, d_hi + 0.5 * step, step)
            if self._clears_profile(handle_c, fed, float(d), tray)
        ]
        if not feasible:
            # WHY did nothing work? Two failures, two different fixes — so check
            # which it was rather than guess.
            point_reaches = any(
                tray.contains(handle_c[:2] + float(d) * fed[:2])
                for d in np.arange(d_lo, d_hi + 0.5 * step, step)
            )
            reason = (
                # The grasp point can sit in the slot, but a fingertip still fouls a
                # LONG rail — the tool must be lying at a slant, not across.
                'the grasp point reaches the opening but a fingertip fouls a long '
                'rail — the tool is lying at a slant rather than across the slot'
                if point_reaches else
                # The grasp point never gets into the slot at all.
                f'no spot in {d_lo * 1000:.0f}..{d_hi * 1000:.0f} mm reaches the '
                f'opening — raise slide_max_m for {det.tool_class}, or the tool '
                f'does not cross the slot at all'
            )
            return self._no_slide(det, nominal_d, shallow_z, reason)

        # Of the safe spots, take the one that sits DEEPEST inside the opening.
        #
        # The tools always lie ACROSS the slot, and the slot is long and narrow, so
        # "deepest inside" is the same thing as "on the slot's centre line, along
        # its short axis" — which is where a flat instrument grips best, and where
        # the fingers are furthest from the profile on both sides.
        #
        # This used to pick the feasible spot NEAREST THE NOMINAL distance, which
        # pulled the grasp toward the handle and therefore toward the NEAR EDGE of
        # the slot. Nominal now only breaks ties, so the choice stays deterministic
        # on a plateau.
        d = max(feasible, key=lambda x: (
            round(tray.clearance(handle_c[:2] + x * fed[:2]), 4),  # 0.1 mm grid vs. float noise
            -abs(x - nominal_d),
        ))
        return d, deep_z, True, 'sliding_opening'

    def _no_slide(self, det, nominal_d, shallow_z, reason):
        """Fall back to the shallow grasp, and say why — throttled, this runs at 4 Hz."""
        self.get_logger().warn(
            f'{det.tool_class} on {det.location}: grasping shallow — {reason}',
            throttle_duration_sec=10.0)
        return nominal_d, shallow_z, False, 'sliding_no_opening'

    def _clears_profile(self, handle_c, fed, distance_m, tray):
        """Can the gripper descend here?

        Two different tests, because the two kinds of edge are different obstacles:

        - The GRASP POINT must stay clear of every edge, short ends included — it is
          the TCP, and the tool has to be there.
        - The FINGERTIPS stick out 32.5 mm on either side, ALONG the slot's long
          axis (the jaws open perpendicular to the tool, and the tools lie across
          the slot). At the short ends the frame is stepped down, so a fingertip may
          hang over one. They must only clear the two long rails.

        Testing the fingers against the short ends as well would sacrifice ~38 mm at
        each end of the slot — on the 217 mm reclaim tray that is a third of it.
        """
        grasp_xy = handle_c[:2] + distance_m * fed[:2]
        if not tray.contains(grasp_xy):
            return False
        fingers = gripper_footprint(
            grasp_xy, fed[:2], self.finger_half_span_m)[1:]
        return all(tray.contains_lateral(f) for f in fingers)


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
