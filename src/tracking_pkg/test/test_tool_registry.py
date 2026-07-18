#!/usr/bin/env python3
"""Offline unit tests for the instrument-count reconciliation. The registry is
deliberately ROS-free, so this runs anywhere:

    python3 -m pytest src/tracking_pkg/test/test_tool_registry.py -q

Covers the reported bug: tools removed by hand during the operation were still
counted as present, because observe() never demotes on absence. reconcile()
turns absence into evidence at count time.
"""

import os
import sys

import pytest

_PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_PKG, 'src', 'reasoning'))

from tool_registry import (  # noqa: E402
    ToolRegistry, INSTRUMENT_TRAY, RECLAIM_TRAY,
    AT_HOME, ON_RECLAIM, IN_GRIPPER, IN_USE, UNKNOWN)


def _obs(tool_class, x, y, track_id='t'):
    return {'tool_class': tool_class, 'track_id': track_id,
            'handle_xy': (x, y), 'end_dir': (1.0, 0.0), 'plane_z': 0.0}


def _registry_with_three(t0=1000.0):
    """Three distinct tools, all registered AT_HOME at t0."""
    reg = ToolRegistry()
    ok, found, _ = reg.register(
        [_obs('awl', 0.0, 0.0, 'a'),
         _obs('hammer', 0.5, 0.0, 'h'),
         _obs('needle_holder', 1.0, 0.0, 'n')],
        now=t0)
    assert ok and sum(found.values()) == 3
    return reg


# ── reconcile: the reported bug ─────────────────────────────────────

def test_reconcile_demotes_the_removed_tool_only():
    reg = _registry_with_three(t0=1000.0)
    # The awl and hammer keep being seen; the needle holder was taken away and
    # stops refreshing. Re-observe the two present ones at t=1010.
    reg.observe(INSTRUMENT_TRAY,
                [_obs('awl', 0.0, 0.0, 'a'), _obs('hammer', 0.5, 0.0, 'h')],
                now=1010.0)

    demoted = reg.reconcile(freshness_sec=2.0, now=1011.0)

    assert demoted == ['needle_holder_1']
    c = reg.count()
    assert c['accounted_for'] == 2
    assert c['expected'] == 3
    assert c['all_accounted_for'] is False
    assert c['unknown'] == ['needle_holder_1']
    assert c['unaccounted_for'] == ['needle_holder_1']


def test_reconcile_keeps_freshly_seen_tools():
    reg = _registry_with_three(t0=1000.0)
    reg.observe(INSTRUMENT_TRAY,
                [_obs('awl', 0.0, 0.0), _obs('hammer', 0.5, 0.0),
                 _obs('needle_holder', 1.0, 0.0)],
                now=1010.0)
    demoted = reg.reconcile(freshness_sec=2.0, now=1010.5)
    assert demoted == []
    assert reg.count()['all_accounted_for'] is True


def test_reconcile_noop_before_freshness_window():
    # All three seen at t0; a count 1 s later (window 2 s) must not demote yet.
    reg = _registry_with_three(t0=1000.0)
    assert reg.reconcile(freshness_sec=2.0, now=1001.0) == []


# ── all_accounted_for now includes reclaim + gripper ────────────────

def test_reclaim_tool_counts_as_accounted():
    reg = _registry_with_three(t0=1000.0)
    # The awl is now on the reclaim tray; the other two stay home. All fresh.
    reg.observe(INSTRUMENT_TRAY,
                [_obs('hammer', 0.5, 0.0), _obs('needle_holder', 1.0, 0.0)],
                now=1010.0)
    reg.observe(RECLAIM_TRAY, [_obs('awl', 9.0, 9.0)], now=1010.0)
    reg.reconcile(freshness_sec=2.0, now=1010.5)

    c = reg.count()
    assert c['on_reclaim'] == ['awl_1']
    assert c['accounted_for'] == 3
    assert c['all_accounted_for'] is True
    assert c['unaccounted_for'] == []


def test_in_use_is_never_demoted_and_stays_unaccounted():
    reg = _registry_with_three(t0=1000.0)
    # Robot handed the awl over: PICKED then HANDED_OVER -> IN_USE.
    reg.on_event('PICKED', 'a', 'awl', now=1005.0)
    reg.on_event('HANDED_OVER', '', 'awl', now=1006.0)
    reg.observe(INSTRUMENT_TRAY,
                [_obs('hammer', 0.5, 0.0), _obs('needle_holder', 1.0, 0.0)],
                now=1010.0)
    reg.reconcile(freshness_sec=2.0, now=1011.0)

    c = reg.count()
    assert c['in_use'] == ['awl_1']          # untouched by reconcile
    assert c['accounted_for'] == 2
    assert c['all_accounted_for'] is False
    assert c['unaccounted_for'] == ['awl_1']


def test_gripper_tool_counts_as_accounted():
    reg = _registry_with_three(t0=1000.0)
    reg.on_event('PICKED', 'a', 'awl', now=1005.0)   # -> IN_GRIPPER
    reg.observe(INSTRUMENT_TRAY,
                [_obs('hammer', 0.5, 0.0), _obs('needle_holder', 1.0, 0.0)],
                now=1010.0)
    reg.reconcile(freshness_sec=2.0, now=1011.0)

    c = reg.count()
    assert c['in_gripper'] == ['awl_1']
    assert c['accounted_for'] == 3
    assert c['all_accounted_for'] is True
