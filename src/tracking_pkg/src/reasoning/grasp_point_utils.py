#!/usr/bin/env python3
"""Shared class-specific tray-tool grasp point helpers."""

import numpy as np


CLASS_OFFSET_M = 0.020
HAMMER_CENTER_OFFSET_M = 0.040


def grasp_rule_for_class(tool_class):
    """Return the class-specific grasp rule name for a model class."""
    cls = (tool_class or "").strip().lower()
    if cls.startswith("hammer"):
        return "hammer_inner_plus"
    if cls.startswith("scissors") or cls.startswith("needle_holder"):
        return "inner_plus"
    return "fallback"


def handle_inner_projection(center_px, corners_px, toward_center_unit):
    """Distance from handle center to the handle edge facing tool center."""
    center = np.asarray(center_px, dtype=float)
    corners = np.asarray(corners_px, dtype=float).reshape(-1, 2)
    direction = np.asarray(toward_center_unit, dtype=float)
    norm = float(np.linalg.norm(direction))
    if norm < 1e-9 or corners.size == 0:
        return 0.0
    direction = direction / norm
    projections = (corners - center) @ direction
    return max(0.0, float(np.max(projections)))


def class_specific_grasp_px(tool_class, handle_center_px, handle_corners_px,
                            axis_unit, sign, fallback_offset_px,
                            class_offset_px):
    """Compute grasp point in image pixels.

    axis_unit is the body OBB long axis. sign is positive when the handle lies
    on the positive side of that axis, so -sign * axis_unit points from the
    handle toward the tool center.
    """
    handle_center = np.asarray(handle_center_px, dtype=float)
    axis = np.asarray(axis_unit, dtype=float)
    axis_norm = float(np.linalg.norm(axis))
    if axis_norm < 1e-9:
        return handle_center.copy(), "fallback"

    toward_center = (-float(sign) * axis) / axis_norm
    rule = grasp_rule_for_class(tool_class)
    if rule == "fallback" or class_offset_px is None or class_offset_px <= 0.0:
        return (
            handle_center + float(fallback_offset_px) * toward_center,
            "fallback",
        )

    inner_px = handle_inner_projection(
        handle_center, handle_corners_px, toward_center,
    )
    distance_px = inner_px + float(class_offset_px)

    return handle_center + distance_px * toward_center, rule


def class_specific_grasp_distance_m(tool_class, inner_edge_distance_m,
                                    fallback_offset_m):
    """Compute distance from handle center toward tool center in meters."""
    rule = grasp_rule_for_class(tool_class)
    if rule == "fallback":
        return float(fallback_offset_m), rule

    inner_m = max(0.0, float(inner_edge_distance_m))
    if rule == "hammer_inner_plus":
        return inner_m + HAMMER_CENTER_OFFSET_M, rule
    return inner_m + CLASS_OFFSET_M, rule
