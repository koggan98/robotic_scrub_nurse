#!/usr/bin/env python3
"""Offline tests for Robotiq gOBJ/gPO grasp detection."""

import os
import sys
from types import SimpleNamespace

import pytest


_PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_PKG, 'src', 'execution'))

from gripper_opener_with_zeroer import (  # noqa: E402
    SocketControllerNode,
    classify_loss_monitor_sample,
    is_confirmed_tool_loss,
    is_tool_grasped,
    validate_loss_confirm_delay,
    validate_rescue_window,
)


class _FakeLogger:
    def __init__(self):
        self.infos = []
        self.warns = []

    def info(self, message):
        self.infos.append(message)

    def warn(self, message):
        self.warns.append(message)


class _FakePublisher:
    def __init__(self):
        self.values = []

    def publish(self, msg):
        self.values.append(msg.data)


class _FakeUR:
    def __init__(self, values):
        self.values = iter(values)

    def query_gripper_var(self, _name):
        return next(self.values)


def _monitor_node(values):
    logger = _FakeLogger()
    publisher = _FakePublisher()
    node = SimpleNamespace(
        monitoring_active=True,
        ur_node=_FakeUR(values),
        grasp_rescue_min_pos=180,
        grasp_rescue_max_pos=228,
        grasp_loss_confirm_delay_sec=0.0,
        tool_grasped_publisher=publisher,
        get_logger=lambda: logger,
    )
    SocketControllerNode._monitor_grasp(node)
    return node, logger, publisher


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


def test_loss_monitor_keeps_gobj3_gpo225_via_same_thin_tool_rescue():
    assert classify_loss_monitor_sample(3, 225, 180, 228) is True


@pytest.mark.parametrize('obj,pos', [(None, None), (0, 225), (0, 230)])
def test_loss_monitor_treats_missing_and_motion_samples_as_inconclusive(obj, pos):
    assert classify_loss_monitor_sample(obj, pos, 180, 228) is None


def test_negative_then_positive_loss_sample_is_ignored():
    first = classify_loss_monitor_sample(3, 230, 180, 228)
    second = classify_loss_monitor_sample(3, 225, 180, 228)
    assert not is_confirmed_tool_loss(first, second)


def test_two_negative_loss_samples_confirm_loss():
    first = classify_loss_monitor_sample(3, 230, 180, 228)
    second = classify_loss_monitor_sample(3, 230, 180, 228)
    assert is_confirmed_tool_loss(first, second)


def test_live_monitor_ignores_negative_then_rescued_positive():
    node, logger, publisher = _monitor_node([3, 230, 3, 225])
    assert node.monitoring_active is True
    assert publisher.values == []
    assert any('transient loss indication ignored' in line
               for line in logger.infos)


def test_live_monitor_publishes_only_after_two_negative_samples():
    node, logger, publisher = _monitor_node([3, 230, 3, 230])
    assert node.monitoring_active is False
    assert publisher.values == [False]
    assert any('confirmed tool loss' in line for line in logger.warns)


def test_live_monitor_does_not_confirm_rescued_gobj3_gpo225():
    node, logger, publisher = _monitor_node([3, 225])
    assert node.monitoring_active is True
    assert publisher.values == []
    assert any('thin-tool rescue still held' in line for line in logger.infos)


def test_valid_rescue_window_is_accepted():
    validate_rescue_window(180, 228, 230)


def test_valid_loss_confirmation_delay_is_accepted():
    validate_loss_confirm_delay(0.0)
    validate_loss_confirm_delay(0.1)


def test_negative_loss_confirmation_delay_is_rejected():
    with pytest.raises(ValueError, match='loss_confirm_delay_sec must be >= 0'):
        validate_loss_confirm_delay(-0.1)


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
