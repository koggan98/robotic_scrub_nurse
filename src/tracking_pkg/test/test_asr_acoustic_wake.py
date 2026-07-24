#!/usr/bin/env python3
"""Offline tests for the acoustic-wake ASR queue and session FSM."""

import os
import queue
import sys
import threading
import time
from threading import Lock
from types import SimpleNamespace

import numpy as np
import pytest


_PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_PKG, 'src', 'interfaces'))

import asr_node  # noqa: E402
from asr_node import ASRNode  # noqa: E402
from wake_word import is_exact_wake_utterance  # noqa: E402


@pytest.fixture(autouse=True)
def _rclpy_ok(monkeypatch):
    monkeypatch.setattr(asr_node.rclpy, 'ok', lambda: True)


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message.data)


class _Detector:
    def __init__(self, scores=None, failure=None, reset_failure=None):
        self.scores = iter(scores or [])
        self.failure = failure
        self.reset_failure = reset_failure
        self.reset_calls = 0

    def process(self, _chunk, _sample_rate):
        if self.failure is not None:
            raise self.failure
        return next(self.scores, None)

    def reset(self):
        self.reset_calls += 1
        if self.reset_failure is not None:
            raise self.reset_failure


def _node(transcripts=(), command_window=2.0):
    node = ASRNode.__new__(ASRNode)
    node._running = True
    node.audio_wake_enabled = True
    node.wake_require_separate_command = True
    node._phase = 'wait_wake'
    node._command_deadline = 0.0
    node._wake_generation = 0
    node._wake_state_lock = Lock()
    node._detector_lock = Lock()
    node._active_segment_generation = None
    node._active_segment_started_at = None
    node._pending_in_time_commands = 0
    node._audio_queue = queue.Queue()
    node._max_queued_segments = 8
    node._wake_detector = _Detector()
    node.capture_sample_rate = 16000
    node.min_speech_seconds = 0.5
    node.max_speech_seconds = 8.0
    node.energy_threshold = 0.1
    node.silence_threshold = 0.35
    node.device_index = None
    node.wake_words = ['robot']
    node.wake_word_fuzzy = 0.75
    node.two_stage_wake = True
    node.command_window_sec = command_window
    node.publisher = _Publisher()
    node.status_publisher = _Publisher()
    node.logs = SimpleNamespace(info=[], warn=[], error=[], debug=[])
    node.get_logger = lambda: SimpleNamespace(
        info=lambda message: node.logs.info.append(message),
        warn=lambda message: node.logs.warn.append(message),
        error=lambda message: node.logs.error.append(message),
        debug=lambda message: node.logs.debug.append(message),
    )
    sequence = iter(transcripts)
    node.transcribe_calls = 0

    def transcribe(_audio):
        node.transcribe_calls += 1
        return next(sequence, None)

    node._transcribe = transcribe
    return node


def _segment():
    # 0.6 s at 16 kHz, split like the live 100-ms capture loop.
    return [np.zeros((1600, 1), dtype=np.float32) for _ in range(6)]


class _ChunkStream:
    def __init__(self, node, chunks, clock):
        self.node = node
        self.chunks = list(chunks)
        self.clock = clock

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return False

    def read(self, _chunk_samples):
        self.clock[0] += 0.1
        chunk = self.chunks.pop(0)
        if not self.chunks:
            self.node._running = False
        return chunk, False


class _SoundDevice:
    def __init__(self, stream):
        self.stream = stream

    def InputStream(self, **_kwargs):
        return self.stream


def _audio_chunk(amplitude):
    return np.full((1600, 1), amplitude, dtype=np.float32)


def _wait_until(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


def _start_worker(node):
    worker = threading.Thread(
        target=node._transcribe_worker, daemon=True)
    worker.start()
    return worker


def _stop_worker(node, worker):
    node._running = False
    worker.join(timeout=2.0)


@pytest.mark.parametrize('value', [
    0.0,
    -1.0,
    float('inf'),
    float('-inf'),
    float('nan'),
    'invalid',
])
def test_positive_finite_timing_parameters_reject_invalid_values(value):
    with pytest.raises(ValueError, match='positive finite'):
        asr_node._positive_finite_parameter('command_window_sec', value)


def test_idle_background_never_enters_whisper_queue():
    node = _node()

    for _ in range(20):
        node._enqueue_segment(_segment())

    assert node._audio_queue.empty()
    assert node.transcribe_calls == 0


def test_new_wake_session_purges_any_stale_queue_before_candidate():
    node = _node()
    for stale in range(7):
        node._audio_queue.put(stale)

    generation = node._begin_wake_verification(0.8)
    assert node._audio_queue.empty()
    node._enqueue_segment(
        _segment(), wake_generation=generation,
        segment_started_at=time.monotonic())

    queued = node._audio_queue.get_nowait()
    assert queued.kind == 'wake'
    assert queued.generation == generation


def test_capture_chunks_split_wake_pause_and_followup_command(monkeypatch):
    node = _node()
    node._wake_detector = _Detector(scores=[None, 0.8])
    clock = [0.0]
    monkeypatch.setattr(asr_node.time, 'monotonic', lambda: clock[0])
    chunks = (
        [_audio_chunk(0.5) for _ in range(3)] +
        [_audio_chunk(0.0) for _ in range(5)] +
        [_audio_chunk(0.5) for _ in range(6)] +
        [_audio_chunk(0.0) for _ in range(5)]
    )
    stream = _ChunkStream(node, chunks, clock)

    node._stream_and_segment(_SoundDevice(stream), chunk_duration=0.1)

    queued = list(node._audio_queue.queue)
    assert [item.kind for item in queued] == ['wake', 'command']
    assert queued[0].generation == queued[1].generation


def test_continuous_wake_candidate_is_bounded_without_silence(monkeypatch):
    node = _node()
    node.max_speech_seconds = 0.5
    node._wake_detector = _Detector(scores=[None, 0.8])
    clock = [0.0]
    monkeypatch.setattr(asr_node.time, 'monotonic', lambda: clock[0])
    stream = _ChunkStream(
        node,
        [_audio_chunk(0.5) for _ in range(8)],
        clock,
    )

    node._stream_and_segment(_SoundDevice(stream), chunk_duration=0.1)

    wake = node._audio_queue.get_nowait()
    assert wake.kind == 'wake'
    assert len(wake.audio) <= int(0.8 * node.capture_sample_rate)
    assert any('Maximum speech segment' in msg for msg in node.logs.warn)


def test_robot_then_command_publishes_and_closes_session():
    node = _node(['Robot.', 'needle holder'])
    generation = node._begin_wake_verification(0.81)
    node._enqueue_segment(
        _segment(), wake_generation=generation,
        segment_started_at=time.monotonic())
    # This is captured while Whisper is still verifying "Robot".
    node._enqueue_segment(
        _segment(), segment_started_at=time.monotonic())

    worker = _start_worker(node)
    assert _wait_until(lambda: node.publisher.messages)
    _stop_worker(node, worker)

    assert node.publisher.messages == ['needle holder']
    assert node.status_publisher.messages == ['listening', '']
    assert node._phase == 'wait_wake'
    assert node.transcribe_calls == 2


@pytest.mark.parametrize('wake_transcript', [
    'Robot needle holder',
    'the robot is over there',
    None,
])
def test_non_exact_wake_candidate_rejects_stale_followers(wake_transcript):
    node = _node([wake_transcript, 'scissors'])
    generation = node._begin_wake_verification(0.72)
    node._enqueue_segment(
        _segment(), wake_generation=generation,
        segment_started_at=time.monotonic())
    node._enqueue_segment(
        _segment(), segment_started_at=time.monotonic())

    worker = _start_worker(node)
    assert _wait_until(lambda: node._phase == 'wait_wake')
    # Let the worker dequeue and discard the tagged follower.
    assert _wait_until(lambda: node._audio_queue.empty())
    _stop_worker(node, worker)

    assert node.publisher.messages == []
    assert node.status_publisher.messages == []
    assert node.transcribe_calls == 1


def test_empty_command_keeps_window_open_for_real_command():
    node = _node(['robot', None, 'awl'])
    generation = node._begin_wake_verification(0.9)
    node._enqueue_segment(
        _segment(), wake_generation=generation,
        segment_started_at=time.monotonic())
    node._enqueue_segment(
        _segment(), segment_started_at=time.monotonic())
    node._enqueue_segment(
        _segment(), segment_started_at=time.monotonic())

    worker = _start_worker(node)
    assert _wait_until(lambda: node.publisher.messages)
    _stop_worker(node, worker)

    assert node.publisher.messages == ['awl']
    assert node.transcribe_calls == 3


def test_command_window_timeout_disarms_and_resets_detector():
    node = _node(['robot'], command_window=0.15)
    generation = node._begin_wake_verification(0.8)
    node._enqueue_segment(
        _segment(), wake_generation=generation,
        segment_started_at=time.monotonic())

    worker = _start_worker(node)
    assert _wait_until(
        lambda: node.status_publisher.messages[-1:] == [''], timeout=1.0)
    _stop_worker(node, worker)

    assert node.publisher.messages == []
    assert node.status_publisher.messages == ['listening', '']
    assert node._phase == 'wait_wake'
    assert node._wake_detector.reset_calls == 1


def test_listening_and_close_status_updates_are_serialized():
    node = _node()
    listening_publish_started = threading.Event()
    release_listening_publish = threading.Event()

    class _BlockingStatusPublisher(_Publisher):
        def publish(self, message):
            super().publish(message)
            if message.data == 'listening':
                listening_publish_started.set()
                release_listening_publish.wait(timeout=2.0)

    node.status_publisher = _BlockingStatusPublisher()
    generation = node._begin_wake_verification(0.8)
    arm_thread = threading.Thread(
        target=node._arm_acoustic_session, args=(generation,))
    arm_thread.start()
    assert listening_publish_started.wait(timeout=2.0)

    close_done = threading.Event()

    def close_session():
        node._close_acoustic_session(generation, 'disconnect')
        close_done.set()

    close_thread = threading.Thread(target=close_session)
    close_thread.start()
    assert not close_done.wait(timeout=0.05)
    release_listening_publish.set()
    arm_thread.join(timeout=2.0)
    close_thread.join(timeout=2.0)

    assert close_done.is_set()
    assert node.status_publisher.messages == ['listening', '']
    assert node._phase == 'wait_wake'


def test_command_that_started_before_deadline_may_finish_after_it():
    node = _node(['robot', 'scissors'], command_window=0.1)
    generation = node._begin_wake_verification(0.8)
    node._enqueue_segment(
        _segment(), wake_generation=generation,
        segment_started_at=time.monotonic())
    worker = _start_worker(node)
    assert _wait_until(lambda: node._phase == 'command')

    started_at = time.monotonic()
    node._note_active_segment_started(started_at)
    time.sleep(0.15)
    node._enqueue_segment(
        _segment(), segment_started_at=started_at)
    node._note_active_segment_finished(started_at)

    assert _wait_until(lambda: node.publisher.messages)
    _stop_worker(node, worker)
    assert node.publisher.messages == ['scissors']


def test_queued_in_time_command_wins_race_with_expiry_check():
    node = _node(['scissors'])
    generation = node._begin_wake_verification(0.8)
    node._arm_acoustic_session(generation)
    node._command_deadline = time.monotonic() - 0.1
    node._enqueue_segment(
        _segment(),
        segment_started_at=node._command_deadline - 0.1,
    )

    # Start the worker only after the deadline and after the capture marker has
    # been cleared. The queued start timestamp must still preserve the command.
    worker = _start_worker(node)
    assert _wait_until(lambda: node.publisher.messages)
    _stop_worker(node, worker)

    assert node.publisher.messages == ['scissors']


def test_in_time_enqueue_between_queue_timeout_and_expiry_is_preserved():
    node = _node(['scissors'], command_window=0.1)
    generation = node._begin_wake_verification(0.8)
    node._arm_acoustic_session(generation)
    node._command_deadline = time.monotonic() - 0.1
    started_at = node._command_deadline - 0.1
    node._note_active_segment_started(started_at)

    timeout_decided = threading.Event()
    release_timeout = threading.Event()

    class _TimeoutRaceQueue(queue.Queue):
        def __init__(self):
            super().__init__()
            self.injected = False

        def get(self, block=True, timeout=None):
            if block and timeout is not None and not self.injected:
                self.injected = True
                timeout_decided.set()
                assert release_timeout.wait(timeout=2.0)
                raise queue.Empty
            return super().get(block=block, timeout=timeout)

    node._audio_queue = _TimeoutRaceQueue()
    worker = _start_worker(node)
    try:
        assert timeout_decided.wait(timeout=2.0)
        node._enqueue_segment(
            _segment(), segment_started_at=started_at)
        node._note_active_segment_finished(started_at)
        release_timeout.set()

        assert _wait_until(lambda: node.publisher.messages, timeout=1.0)
    finally:
        release_timeout.set()
        _stop_worker(node, worker)

    assert node.publisher.messages == ['scissors']
    assert node.transcribe_calls == 1
    assert node._pending_in_time_commands == 0
    assert not any(
        'command window expired' in message for message in node.logs.info)


def test_session_closed_during_transcription_never_publishes_stale_command():
    node = _node()
    command_started = threading.Event()
    release_command = threading.Event()
    calls = 0

    def transcribe(_audio):
        nonlocal calls
        calls += 1
        if calls == 1:
            return 'robot'
        command_started.set()
        release_command.wait(timeout=2.0)
        return 'needle holder'

    node._transcribe = transcribe
    generation = node._begin_wake_verification(0.8)
    node._enqueue_segment(
        _segment(), wake_generation=generation,
        segment_started_at=time.monotonic())
    node._enqueue_segment(
        _segment(), segment_started_at=time.monotonic())
    worker = _start_worker(node)
    assert command_started.wait(timeout=2.0)

    assert node._close_acoustic_session(
        generation, 'microphone disconnected')
    release_command.set()
    assert _wait_until(lambda: node._audio_queue.empty())
    _stop_worker(node, worker)

    assert node.publisher.messages == []


def test_runtime_detector_error_stops_instead_of_falling_back(monkeypatch):
    node = _node()
    node._wake_detector = _Detector(failure=RuntimeError('broken model'))
    shutdown_calls = []
    monkeypatch.setattr(
        asr_node.rclpy, 'shutdown', lambda: shutdown_calls.append(True))

    assert node._process_acoustic_wake(
        np.zeros((1600, 1), dtype=np.float32)) is None
    assert not node._running
    assert 'broken model' in node._fatal_error
    assert shutdown_calls == [True]
    assert any('ASR is stopping' in message for message in node.logs.error)


def test_detector_reset_error_also_fails_closed(monkeypatch):
    node = _node()
    node._wake_detector = _Detector(
        reset_failure=RuntimeError('reset failed'))
    shutdown_calls = []
    monkeypatch.setattr(
        asr_node.rclpy, 'shutdown', lambda: shutdown_calls.append(True))
    generation = node._begin_wake_verification(0.8)

    assert not node._close_acoustic_session(generation, 'test close')
    assert not node._running
    assert 'reset failed' in node._fatal_error
    assert shutdown_calls == [True]


def test_closed_session_cannot_retrigger_while_detector_reset_is_running():
    node = _node()
    reset_started = threading.Event()
    release_reset = threading.Event()

    def blocking_reset():
        reset_started.set()
        assert release_reset.wait(timeout=2.0)

    node._wake_detector.reset = blocking_reset
    generation = node._begin_wake_verification(0.8)
    node._arm_acoustic_session(generation)
    close_thread = threading.Thread(
        target=node._close_acoustic_session,
        args=(generation, 'command accepted'),
    )
    close_thread.start()
    try:
        assert reset_started.wait(timeout=2.0)
        assert node._phase == 'resetting'
        assert node._process_acoustic_wake(
            np.zeros((1600, 1), dtype=np.float32)) is None
        assert node._wake_generation == generation + 1
    finally:
        release_reset.set()
        close_thread.join(timeout=2.0)

    assert not close_thread.is_alive()
    assert node._phase == 'wait_wake'
    assert node._wake_generation == generation + 1


def test_generic_capture_error_clears_active_session_before_exit():
    node = _node()
    generation = node._begin_wake_verification(0.8)
    node._arm_acoustic_session(generation)
    node._note_active_segment_started(time.monotonic())

    def fail_stream(_sd, _duration):
        node._running = False
        raise RuntimeError('stream failed')

    node._stream_and_segment = fail_stream
    sounddevice = SimpleNamespace(
        PortAudioError=type('PortAudioError', (Exception,), {}))

    node._capture_loop(sounddevice)

    assert node._phase == 'wait_wake'
    assert node._active_segment_started_at is None
    assert node._wake_detector.reset_calls == 1
    assert node.status_publisher.messages == ['listening', '']


def test_microphone_hot_swap_resets_active_detector_and_session():
    node = _node()
    node.device_index = 3
    node.capture_sample_rate = 16000
    generation = node._begin_wake_verification(0.8)
    node._arm_acoustic_session(generation)

    node._apply_audio_selection(
        (7, 'USB Composite Device', 48000, 'USB Composite Device'))

    assert node.device_index == 7
    assert node.capture_sample_rate == 48000
    assert node._phase == 'wait_wake'
    assert node._wake_detector.reset_calls == 1
    assert node.status_publisher.messages == ['listening', '']


def test_missing_model_fails_closed_before_audio_threads_start(tmp_path):
    node = _node()
    node.audio_wake_model_path = str(tmp_path / 'missing-robot.onnx')
    node.audio_wake_threshold = 0.5

    with pytest.raises(RuntimeError, match='does not exist'):
        node._create_wake_detector()

    assert any('does not exist' in message for message in node.logs.error)


def test_unloadable_wake_library_fails_closed_before_audio_threads_start(
        monkeypatch, tmp_path):
    node = _node()
    model_path = tmp_path / 'robot.onnx'
    model_path.write_bytes(b'test model')
    node.audio_wake_model_path = str(model_path)
    node.audio_wake_threshold = 0.5

    class _UnavailableDetector:
        def __init__(self, *_args, **_kwargs):
            raise ImportError('openwakeword unavailable')

    monkeypatch.setitem(
        sys.modules,
        'openwakeword_detector',
        SimpleNamespace(OpenWakeWordDetector=_UnavailableDetector),
    )

    with pytest.raises(RuntimeError, match='Failed to initialize'):
        node._create_wake_detector()

    assert node.transcribe_calls == 0
    assert any(
        'openwakeword unavailable' in message
        for message in node.logs.error)


@pytest.mark.parametrize('text,expected', [
    ('Robot.', True),
    ('robot', True),
    ('Robot needle holder', False),
    ('hey robot', False),
    ('the robot', False),
    ('robot robot', False),
    (None, False),
])
def test_exact_wake_verification(text, expected):
    assert is_exact_wake_utterance(text, ['robot']) is expected
