#!/usr/bin/env python3
"""The transcription worker of the continuous-capture ASR. No mic / no ROS
spin: we drive the queue directly and stub Whisper, verifying the two-stage
wake handling that used to live in the (now split) capture loop.

The headline guarantee: a command spoken right after the wake word — captured
into the queue while the wake word is still being transcribed — is NOT lost.
"""

import os
import queue
import sys
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

_PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_PKG, 'src', 'interfaces'))

import asr_node  # noqa: E402
from asr_node import ASRNode  # noqa: E402


@pytest.fixture(autouse=True)
def _rclpy_ok(monkeypatch):
    # The worker loops on `rclpy.ok()`, which is False without rclpy.init();
    # termination is driven by node._running here instead.
    monkeypatch.setattr(asr_node.rclpy, 'ok', lambda: True)


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m.data)


def _worker_node(transcripts, command_window_sec=2.0):
    """A bare ASRNode wired just enough to run _transcribe_worker, with Whisper
    stubbed to return `transcripts[i]` for the i-th queued segment."""
    node = ASRNode.__new__(ASRNode)
    node._running = True
    node._phase = 'wait_wake'
    node._command_deadline = 0.0
    node._audio_queue = queue.Queue()
    node._max_queued_segments = 8
    node.wake_words = ['robot', 'robo', 'rob', 'robi']
    node.wake_word_fuzzy = 0.75
    node.two_stage_wake = True
    node.command_window_sec = command_window_sec
    node.publisher = _Pub()
    node.status_publisher = _Pub()
    node.get_logger = lambda: SimpleNamespace(
        info=lambda *a: None, warn=lambda *a: None,
        error=lambda *a: None, debug=lambda *a: None)
    seq = list(transcripts)
    node._transcribe = lambda _audio: seq.pop(0) if seq else None
    return node


def _seg():
    return np.zeros(1600, dtype='float32')


def _wait_until(pred, timeout):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if pred():
            return True
        time.sleep(0.01)
    return pred()


def _run(node, timeout=2.0, until=None):
    t = threading.Thread(target=node._transcribe_worker, daemon=True)
    t.start()
    if until is not None:
        _wait_until(until, timeout)
    else:
        time.sleep(timeout)
    node._running = False
    t.join(timeout=2.0)


def test_command_right_after_wake_is_not_lost():
    # Both segments queued (as continuous capture would, while the wake word is
    # still decoding). The command must still be published.
    node = _worker_node(['robot', 'needle holder'])
    node._audio_queue.put(_seg())     # -> "robot"  -> arm
    node._audio_queue.put(_seg())     # -> command  (captured meanwhile)
    _run(node, until=lambda: node.publisher.msgs)
    assert node.publisher.msgs == ['needle holder']
    assert 'listening' in node.status_publisher.msgs   # armed feedback fired
    assert node._phase == 'wait_wake'                  # disarmed after command


def test_one_breath_command_publishes_directly():
    node = _worker_node(['robot needle holder'])
    node._audio_queue.put(_seg())
    _run(node, until=lambda: node.publisher.msgs)
    assert node.publisher.msgs == ['needle holder']


def test_empty_segment_does_not_disarm_the_window():
    # An empty/garbage segment (e.g. a cough) inside the command window must not
    # consume it — the real command that follows is still published.
    node = _worker_node(['robot', None, 'awl'])
    for _ in range(3):
        node._audio_queue.put(_seg())
    _run(node, until=lambda: node.publisher.msgs)
    assert node.publisher.msgs == ['awl']


def test_command_window_timeout_disarms():
    # Wake word, then nothing: after the window the node disarms, publishes no
    # command, and its status returns to idle.
    node = _worker_node(['robot'], command_window_sec=0.3)
    node._audio_queue.put(_seg())
    _run(node, timeout=1.0,
         until=lambda: node.status_publisher.msgs[-1:] == [''])
    assert node.publisher.msgs == []
    assert node.status_publisher.msgs[0] == 'listening'
    assert node.status_publisher.msgs[-1] == ''
    assert node._phase == 'wait_wake'


def test_no_wake_word_is_ignored():
    node = _worker_node(['the tray looks fine'])
    node._audio_queue.put(_seg())
    _run(node, timeout=0.4)
    assert node.publisher.msgs == []
