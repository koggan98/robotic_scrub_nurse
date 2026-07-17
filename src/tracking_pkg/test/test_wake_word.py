#!/usr/bin/env python3
"""Offline unit tests for the wake-word gate. No ROS required:

    python3 -m pytest src/tracking_pkg/test/test_wake_word.py -q
"""

import os
import sys

import pytest

_PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_PKG, 'src', 'interfaces'))

from wake_word import strip_wake_word  # noqa: E402

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
