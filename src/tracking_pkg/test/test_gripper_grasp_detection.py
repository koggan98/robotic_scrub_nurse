#!/usr/bin/env python3
"""Offline tests for Robotiq gOBJ/gPO grasp detection."""

import os
import sys

import pytest


_PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_PKG, 'src', 'execution'))

from gripper_opener_with_zeroer import (  # noqa: E402
    is_tool_grasped,
    validate_rescue_window,
)


@pytest.mark.parametrize('position,expected', [
    (100, False),  # open gripper
    (179, False),
    (180, True),
    (227, True),
    (228, False),  # exclusive upper bound
    (230, False),  # approximately empty full close
])
def test_thin_tool_rescue_boundaries(position, expected):
    assert is_tool_grasped(3, position, 180, 228) is expected


def test_robotiq_contact_detection_wins_outside_rescue_window():
    assert is_tool_grasped(2, None, 180, 228)
    assert is_tool_grasped(2, 230, 180, 228)


def test_missing_position_is_not_rescued_without_contact():
    assert not is_tool_grasped(None, None, 180, 228)


def test_valid_rescue_window_is_accepted():
    validate_rescue_window(180, 228, 230)


@pytest.mark.parametrize('rescue_min,rescue_max,empty_close', [
    (-1, 228, 230),
    (180, 180, 230),
    (228, 228, 230),
    (180, 230, 230),
    (180, 231, 230),
    (180, 228, 256),
])
def test_invalid_rescue_windows_are_rejected(
        rescue_min, rescue_max, empty_close):
    with pytest.raises(ValueError, match='expected 0 <= rescue_min_pos'):
        validate_rescue_window(rescue_min, rescue_max, empty_close)
