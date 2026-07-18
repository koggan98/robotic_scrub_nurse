#!/usr/bin/env python3
"""Offline unit tests for the HRI traffic-light priority resolution. No ROS/cv2:

    python3 -m pytest src/tracking_pkg/test/test_hri_display_logic.py -q
"""

import os
import sys

import pytest

_PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_PKG, 'src', 'interfaces'))

from hri_display_logic import (  # noqa: E402
    resolve_display, is_alert_response, pretty_tool, Debouncer, wrap_lines)


def _r(alert_active=False, alert_text='', system_state='IDLE',
       active_tool_class='', last_response='', handover_waiting=False,
       asr_listening=False, pulse_take=True):
    return resolve_display(alert_active, alert_text, system_state,
                           active_tool_class, last_response, handover_waiting,
                           asr_listening, pulse_take)


# ── Colour mapping per state ────────────────────────────────────────

def test_idle_is_green_ready():
    assert _r(system_state='IDLE') == ('green', 'READY', 'Ready', 'steady')


def test_idle_shows_last_response():
    color, head, detail, _ = _r(system_state='IDLE',
                                last_response='No needle holder on tray.')
    assert color == 'green'
    assert detail == 'No needle holder on tray.'


@pytest.mark.parametrize('state', ['PICKING', 'TRANSPORTING', 'RETURNING',
                                   'RELEASING', 'HANDOVER'])
def test_moving_states_are_red(state):
    color, head, _, anim = _r(system_state=state)
    assert color == 'red' and head == 'BUSY' and anim == 'steady'


def test_presenting_is_green_take_with_tool():
    color, head, detail, anim = _r(system_state='PRESENTING',
                                   active_tool_class='needle_holder')
    assert color == 'green' and head == 'TAKE'
    assert detail == 'Take Needle Holder' and anim == 'pulse'


def test_take_pulse_can_be_disabled():
    _, _, _, anim = _r(system_state='PRESENTING', active_tool_class='awl',
                       pulse_take=False)
    assert anim == 'steady'


def test_gesture_wait_is_amber():
    assert _r(handover_waiting=True) == (
        'amber', 'GESTURE', 'Make your gesture', 'steady')


def test_await_gesture_state_is_amber():
    # The executor's AWAIT_GESTURE state must show amber even though the last
    # movement state (before it) was red — this is the reported "BUSY while
    # waiting for the gesture" bug.
    assert _r(system_state='AWAIT_GESTURE') == (
        'amber', 'GESTURE', 'Make your gesture', 'steady')


def test_listening_is_amber():
    assert _r(asr_listening=True) == (
        'amber', 'SPEAK', 'Speak now', 'steady')


def test_alert_blinks_red_over_everything():
    # Alert wins even while moving and while a tool is named.
    color, head, detail, anim = _r(
        alert_active=True, alert_text='Gesture out of reach',
        system_state='TRANSPORTING', active_tool_class='awl')
    assert color == 'red' and head == 'ALERT'
    assert detail == 'Gesture out of reach' and anim == 'blink'


def test_recovery_error_is_a_persistent_red_recovery_prompt():
    assert _r(system_state='RECOVERY_ERROR') == (
        'red', 'RECOVERY', 'Send robot home', 'blink')


def test_holding_recovery_prompts_operator_to_return_tool():
    assert _r(
        system_state='RECOVERY_ERROR', active_tool_class='hammer') == (
        'red', 'RECOVERY', 'Hammer held — return tool', 'blink')


# ── Priority ordering ───────────────────────────────────────────────

def test_moving_beats_gesture_and_listening():
    # If somehow both are set, arm motion (hazard) must win.
    color, head, _, _ = _r(system_state='PICKING', handover_waiting=True,
                           asr_listening=True)
    assert color == 'red'


def test_gesture_beats_listening():
    _, head, _, _ = _r(handover_waiting=True, asr_listening=True)
    assert head == 'GESTURE'


def test_returning_detail():
    _, _, detail, _ = _r(system_state='RETURNING', active_tool_class='awl')
    assert detail == 'Returning'


# ── Alert classification ────────────────────────────────────────────

@pytest.mark.parametrize('text', [
    'Cannot pick. Tool keeps dropping.',
    'Dropped it. Retrying.',
    'Missed. Retrying.',
    'Count service unavailable.',
    'Gesture out of reach',
])
def test_alert_responses(text):
    assert is_alert_response(text) is True


@pytest.mark.parametrize('text', [
    'No needle holder on tray.',   # info, not an alarm
    'Which one? Long or short?',
    'Needle holder. Picking.',
    'All 3 accounted for.',
    'Nothing held.',
])
def test_non_alert_responses(text):
    assert is_alert_response(text) is False


def test_pretty_tool():
    assert pretty_tool('needle_holder') == 'Needle Holder'
    assert pretty_tool('scissors_long') == 'Scissors Long'
    assert pretty_tool('') == ''


# ── Text wrapping (the register response overflow) ──────────────────

def test_wrap_short_text_single_line():
    assert wrap_lines('Ready', 20) == ['Ready']


def test_wrap_empty():
    assert wrap_lines('', 20) == []
    assert wrap_lines('   ', 20) == []


def test_wrap_register_response_multiline():
    text = 'Registered 2. Needle Holder, Long Scissors.'
    lines = wrap_lines(text, 20, max_lines=3)
    assert len(lines) >= 2
    assert all(len(ln) <= 21 for ln in lines)          # <= max + ellipsis
    # No word is lost across the (fitting) wrap.
    assert 'Needle' in ' '.join(lines)
    assert 'Scissors' in ' '.join(lines)


def test_wrap_overflow_gets_ellipsis():
    text = 'one two three four five six seven eight nine ten eleven twelve'
    lines = wrap_lines(text, 12, max_lines=2)
    assert len(lines) == 2
    assert lines[-1].endswith('…')


# ── Debouncer ───────────────────────────────────────────────────────

GREEN = ('green', 'READY', 'Ready', 'steady')
AMBER = ('amber', 'SPEAK', 'Speak now', 'steady')
RED = ('red', 'BUSY', 'Fetching Awl', 'steady')
ALERT = ('red', 'ALERT', 'Dropped', 'blink')


def test_debounce_first_commit_immediate():
    d = Debouncer(0.35)
    assert d.commit(GREEN, now=100.0) == GREEN


def test_debounce_calm_change_waits():
    d = Debouncer(0.35)
    d.commit(GREEN, now=100.0)
    # Amber appears; within the window the shown state stays green.
    assert d.commit(AMBER, now=100.1) == GREEN
    assert d.commit(AMBER, now=100.2) == GREEN
    # After debounce_sec of stable amber, it commits.
    assert d.commit(AMBER, now=100.5) == AMBER


def test_debounce_red_is_immediate():
    d = Debouncer(0.35)
    d.commit(GREEN, now=100.0)
    assert d.commit(RED, now=100.01) == RED       # safety: no wait


def test_debounce_alert_is_immediate():
    d = Debouncer(0.35)
    d.commit(GREEN, now=100.0)
    assert d.commit(ALERT, now=100.01) == ALERT


def test_debounce_flicker_back_resets_timer():
    d = Debouncer(0.35)
    d.commit(GREEN, now=100.0)
    d.commit(AMBER, now=100.1)      # pending amber
    d.commit(GREEN, now=100.2)      # back to green (== shown) -> resets
    # Amber must again wait the full window from its new first-seen.
    assert d.commit(AMBER, now=100.3) == GREEN
    assert d.commit(AMBER, now=100.66) == AMBER
