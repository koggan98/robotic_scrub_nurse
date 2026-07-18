#!/usr/bin/env python3
"""Offline unit tests for the wake-word gate. No ROS required:

    python3 -m pytest src/tracking_pkg/test/test_wake_word.py -q
"""

import os
import sys

import pytest

_PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_PKG, 'src', 'interfaces'))

from wake_word import strip_wake_word, plan_wake_segment  # noqa: E402

WAKE = ['robot', 'robo', 'rob', 'robi', 'robbie', 'robert']


# ── Wake word heard -> command passes, wake word stripped ───────────

@pytest.mark.parametrize('text,command', [
    ('robot give me the scissors', 'give me the scissors'),
    ('Robot, needle holder please.', 'needle holder please.'),
    ('Robbie scissors', 'scissors'),                 # Whisper's "robi"
    ('Roby, wrong tool', 'wrong tool'),              # fuzzy vs "robi"/"roby"
    ('Robert count the instruments', 'count the instruments'),
    ('hey robot stop', 'stop'),                      # filler in front
    ('Ok, Robot, awl back', 'awl back'),
    ('Robots, hammer', 'hammer'),                    # plural re-spelling
])
def test_wake_word_strips(text, command):
    assert strip_wake_word(text, WAKE) == command


def test_wake_word_only_returns_empty():
    assert strip_wake_word('Robot.', WAKE) == ''


# ── No wake word -> gate closed ─────────────────────────────────────

@pytest.mark.parametrize('text', [
    'give me the scissors',           # a command, but not addressed to us
    'needle holder',
    'problem with the tray',          # "problem" must not fuzzy-match "rob"
    'Thank you.',                     # classic Whisper noise hallucination
    'the robot is over there',        # ABOUT the robot, not addressed to it
])
def test_no_wake_word_blocks(text):
    assert strip_wake_word(text, WAKE) is None


# ── Disabled gate lets everything through ───────────────────────────

def test_empty_wake_list_disables_gate():
    assert strip_wake_word('needle holder', []) == 'needle holder'


# ── Two-stage wake FSM planner ──────────────────────────────────────

def test_plan_wake_only_arms():
    assert plan_wake_segment('wait_wake', 'Robot.', WAKE) == ('arm',)


def test_plan_wake_only_ignored_when_single_stage():
    assert plan_wake_segment('wait_wake', 'Robot.', WAKE,
                             two_stage=False) == ('ignore', None)


def test_plan_one_breath_publishes_command():
    assert plan_wake_segment('wait_wake', 'robot needle holder', WAKE) == (
        ('command', 'needle holder'))


def test_plan_no_wake_word_ignored():
    assert plan_wake_segment('wait_wake', 'needle holder', WAKE) == (
        ('ignore', None))


def test_plan_command_phase_publishes_segment():
    assert plan_wake_segment('command', 'needle holder', WAKE) == (
        ('command', 'needle holder'))


def test_plan_command_phase_strips_repeated_wake():
    assert plan_wake_segment('command', 'robot scissors', WAKE) == (
        ('command', 'scissors'))


@pytest.mark.parametrize('phase,text', [
    ('wait_wake', 'Robot, Oh.'),
    ('command', 'Oh.'),
    ('command', 'robot oh'),
])
def test_addressed_exact_oh_is_corrected_to_awl(phase, text):
    assert plan_wake_segment(phase, text, WAKE) == ('command', 'awl')


def test_oh_correction_is_narrow_and_requires_wake_gate():
    assert plan_wake_segment('command', 'oh please', WAKE) == (
        'command', 'oh please')
    assert plan_wake_segment('wait_wake', 'Oh.', WAKE) == ('ignore', None)
    assert plan_wake_segment('wait_wake', 'Oh.', []) == ('command', 'Oh.')


def test_plan_command_phase_timeout_disarms():
    assert plan_wake_segment('command', None, WAKE) == ('disarm', None)


def test_plan_gate_disabled_passes_everything():
    assert plan_wake_segment('wait_wake', 'needle holder', []) == (
        ('command', 'needle holder'))
