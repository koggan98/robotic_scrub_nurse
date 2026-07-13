#!/usr/bin/env python3
"""Tray opening geometry: is a world-XY point safely inside the tray's hole?

The trays are ITEM extrusion frames — a rectangle of profile bars with an open
hole in the middle. Tools lie across the frame, spanning the hole. Over the hole
the gripper may descend below the tray surface; over a profile bar it must not.

NEITHER tray's support surface is a MoveIt collision object, so MoveIt will not
stop a descent onto a bar. The containment test in here is the ONLY thing between
a deep grasp and a crash into the profile. Be conservative.

Geometry is static: the trays are bolted down and the camera poses are ArUco-
locked once and then frozen, so the openings are constants in the world frame.
Measured with ros_unrelated_scripts/tray_opening_calib.py into
config/tray_geometry.yaml.
"""

import numpy as np


class TrayOpenings:
    """The openings of one tray, as world-XY polygons."""

    def __init__(self, plane_z, polygons, edge_margin_m=0.008):
        self.plane_z = float(plane_z)
        self.edge_margin_m = float(edge_margin_m)
        # Each polygon: (N, 2) float array of world XY, in order around the rim.
        self.polygons = [
            np.asarray(p, dtype=float).reshape(-1, 2)
            for p in (polygons or [])
            if len(p) >= 3
        ]

    def __bool__(self):
        """False when no opening is known — callers must then stay shallow."""
        return bool(self.polygons)

    def contains(self, point_xy, margin_m=None):
        """True if the point is inside some opening AND at least margin_m away
        from every edge of it.

        The margin is applied as a distance-to-edge test rather than by shrinking
        the polygon: a uniform inset by the gripper's finger span would collapse a
        small opening to nothing, while this still admits the middle of it.
        """
        margin = self.edge_margin_m if margin_m is None else float(margin_m)
        p = np.asarray(point_xy, dtype=float)[:2]
        for poly in self.polygons:
            if _point_in_polygon(p, poly) and _distance_to_boundary(p, poly) >= margin:
                return True
        return False

    def clearance(self, point_xy):
        """How deep inside an opening the point sits. 0.0 if it is in none of them.

        This is what "grasp as centrally as possible" reduces to. The openings are
        long NARROW slots and the tools always lie ACROSS them, so the distance to
        the boundary is dominated by the two long edges — maximising it puts the
        point on the slot's centre line, along its short axis. Exactly where a flat
        instrument should be grasped. The largest possible clearance from the
        profile comes along for free.
        """
        p = np.asarray(point_xy, dtype=float)[:2]
        best = 0.0
        for poly in self.polygons:
            if _point_in_polygon(p, poly):
                best = max(best, _distance_to_boundary(p, poly))
        return best

    @classmethod
    def from_config(cls, tray_cfg):
        """Build from one tray's block of config/tray_geometry.yaml."""
        tray_cfg = tray_cfg or {}
        polys = [
            o.get('polygon', [])
            for o in (tray_cfg.get('openings') or [])
        ]
        return cls(
            plane_z=tray_cfg.get('plane_z', 0.0),
            polygons=polys,
            edge_margin_m=tray_cfg.get('edge_margin_m', 0.008),
        )


def _point_in_polygon(point, poly):
    """Even-odd ray crossing. Works for any simple polygon, convex or not."""
    x, y = float(point[0]), float(point[1])
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        # Does the edge j->i straddle the horizontal ray to +x from the point?
        if (yi > y) != (yj > y):
            x_cross = xi + (y - yi) * (xj - xi) / (yj - yi)
            if x_cross > x:
                inside = not inside
        j = i
    return inside


def _distance_to_boundary(point, poly):
    """Shortest distance from the point to any edge of the polygon."""
    p = np.asarray(point, dtype=float)[:2]
    a = poly                      # edge starts
    b = np.roll(poly, -1, axis=0)  # edge ends
    ab = b - a
    ap = p - a
    denom = np.einsum('ij,ij->i', ab, ab)
    # Degenerate (zero-length) edges: clamp t to 0 so we measure to the vertex.
    t = np.divide(
        np.einsum('ij,ij->i', ap, ab), denom,
        out=np.zeros(len(poly)), where=denom > 1e-12,
    )
    t = np.clip(t, 0.0, 1.0)
    closest = a + t[:, None] * ab
    return float(np.min(np.linalg.norm(p - closest, axis=1)))


def gripper_footprint(grasp_xy, tool_axis_xy, finger_half_span_m):
    """The three points that must clear the profile when the gripper descends.

    The executor builds the grasp orientation with tool_yaw_offset_rad = pi/2
    (topDownQuaternionFromHandleAxis), so the jaws open PERPENDICULAR to the
    tool's long axis: the two fingers come down on either side of the tool,
    offset across it. Those two fingertips — not just the grasp point — are what
    would hit a profile bar.
    """
    g = np.asarray(grasp_xy, dtype=float)[:2]
    axis = np.asarray(tool_axis_xy, dtype=float)[:2]
    n = float(np.linalg.norm(axis))
    if n < 1e-9:
        return [g]
    axis = axis / n
    perp = np.array([-axis[1], axis[0]])
    h = float(finger_half_span_m)
    return [g, g + h * perp, g - h * perp]
