#!/usr/bin/env python3
"""Offline tests for the Jetson launch microphone profiles."""

import importlib.util
import os

import pytest


_PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_LAUNCH_PATH = os.path.join(_PKG, 'launch', 'jetson_launch.py')
_SPEC = importlib.util.spec_from_file_location('jetson_launch', _LAUNCH_PATH)
jetson_launch = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(jetson_launch)


@pytest.mark.parametrize('profile,expected', [
    ('samson', ('Samson', 16000)),
    ('SAMSON', ('Samson', 16000)),
    ('jieli', ('USB Composite Device', 48000)),
    (' default ', ('', 16000)),
])
def test_resolve_microphone_profile(profile, expected):
    assert jetson_launch._resolve_microphone_profile(profile) == expected


def test_unknown_microphone_profile_is_rejected_with_choices():
    with pytest.raises(ValueError) as exc_info:
        jetson_launch._resolve_microphone_profile('unknown')

    message = str(exc_info.value)
    assert 'samson' in message
    assert 'jieli' in message
    assert 'default' in message
