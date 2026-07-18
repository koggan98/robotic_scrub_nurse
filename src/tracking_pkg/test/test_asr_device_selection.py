#!/usr/bin/env python3
"""Offline tests for automatic ASR microphone selection."""

import os
import sys

import pytest


_PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_PKG, 'src', 'interfaces'))

import asr_node  # noqa: E402
from asr_node import (  # noqa: E402
    ASRNode,
    AudioDeviceSelectionError,
    audio_capture_block_size,
    select_supported_audio_device,
)


CANDIDATES = ['Samson', 'USB Composite Device']
RATES = [16000, 48000]


def _device(name, inputs):
    return {'name': name, 'max_input_channels': inputs}


class _Logger:
    def __init__(self):
        self.info_messages = []
        self.warn_messages = []
        self.error_messages = []

    def info(self, message):
        self.info_messages.append(message)

    def warn(self, message):
        self.warn_messages.append(message)

    def error(self, message):
        self.error_messages.append(message)


class _ChangingSoundDevice:
    def __init__(self, device_lists):
        self.device_lists = list(device_lists)
        self.refresh_count = 0

    def query_devices(self):
        if len(self.device_lists) > 1:
            return self.device_lists.pop(0)
        return self.device_lists[0]

    def _terminate(self):
        self.refresh_count += 1

    def _initialize(self):
        pass


def _automatic_node():
    node = ASRNode.__new__(ASRNode)
    node._auto_select_audio = True
    node.audio_device_candidates = CANDIDATES
    node.audio_device_candidate_rates = RATES
    node.audio_device_retry_seconds = 5.0
    node._last_audio_detection_issue = None
    node._running = True
    node.device_index = None
    node.capture_sample_rate = 16000
    node.logger = _Logger()
    node.get_logger = lambda: node.logger
    return node


def test_selects_samson_and_its_native_rate():
    devices = [
        _device('HDA Intel HDMI', 0),
        _device('Samson Q2U Microphone: USB Audio (hw:1,0)', 1),
    ]

    assert select_supported_audio_device(devices, CANDIDATES, RATES) == (
        1, 'Samson Q2U Microphone: USB Audio (hw:1,0)', 16000, 'Samson')


def test_selects_jieli_and_ignores_internal_and_output_only_devices():
    devices = [
        _device('HDA Intel PCH: HDMI', 0),
        _device('Built-in Audio Analog Stereo', 2),
        _device('USB Composite Device: Audio (hw:2,0)', 1),
    ]

    assert select_supported_audio_device(devices, CANDIDATES, RATES) == (
        2, 'USB Composite Device: Audio (hw:2,0)', 48000,
        'USB Composite Device')


def test_no_supported_microphone_returns_waiting_state():
    devices = [_device('Built-in Audio Analog Stereo', 2)]

    assert select_supported_audio_device(devices, CANDIDATES, RATES) is None


def test_waits_without_microphone_then_detects_new_jieli(monkeypatch):
    no_microphone = [_device('Built-in Audio Analog Stereo', 2)]
    jieli = [_device('USB Composite Device: Audio (hw:1,0)', 1)]
    sounddevice = _ChangingSoundDevice([no_microphone, jieli])
    node = _automatic_node()
    monkeypatch.setattr(asr_node.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(asr_node.time, 'sleep', lambda _seconds: None)

    assert node._wait_for_audio_input(sounddevice)
    assert node.device_index == 0
    assert node.capture_sample_rate == 48000
    assert sounddevice.refresh_count == 1
    assert any('No supported microphone' in message
               for message in node.logger.warn_messages)


def test_two_supported_microphones_are_rejected():
    devices = [
        _device('Samson Q2U Microphone', 1),
        _device('USB Composite Device: Audio', 1),
    ]

    with pytest.raises(AudioDeviceSelectionError, match='multiple supported'):
        select_supported_audio_device(devices, CANDIDATES, RATES)


def test_candidate_configuration_lengths_must_match():
    with pytest.raises(AudioDeviceSelectionError, match='equal length'):
        select_supported_audio_device([], CANDIDATES, [16000])


def test_hot_swap_updates_device_rate_and_block_size():
    node = _automatic_node()

    node._apply_audio_selection((3, 'Samson Q2U', 16000, 'Samson'))
    assert node.device_index == 3
    assert node.capture_sample_rate == 16000
    assert audio_capture_block_size(node.capture_sample_rate, 0.1) == 1600

    node._apply_audio_selection((7, 'USB Composite Device', 48000,
                                 'USB Composite Device'))
    assert node.device_index == 7
    assert node.capture_sample_rate == 48000
    assert audio_capture_block_size(node.capture_sample_rate, 0.1) == 4800
