#!/usr/bin/env python3
"""Offline tests for the streaming openWakeWord adapter."""

import os
import sys

import numpy as np
import pytest


_PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_PKG, 'src', 'interfaces'))

from openwakeword_detector import (  # noqa: E402
    OpenWakeWordDetector,
)


class _FakeModel:
    def __init__(self, scores=None):
        self.scores = iter(scores or [])
        self.frames = []
        self.reset_calls = 0

    def predict(self, frame):
        self.frames.append(frame.copy())
        try:
            score = next(self.scores)
        except StopIteration:
            score = 0.0
        return {'robot': score}

    def reset(self):
        self.reset_calls += 1


def _detector(tmp_path, scores=None, threshold=0.5):
    tmp_path.mkdir(parents=True, exist_ok=True)
    model_path = tmp_path / 'robot.onnx'
    model_path.write_bytes(b'fake model')
    model = _FakeModel(scores)
    factory_calls = []

    def factory(**kwargs):
        factory_calls.append(kwargs)
        return model

    detector = OpenWakeWordDetector(
        model_path,
        threshold,
        model_factory=factory,
    )
    return detector, model, factory_calls


def test_factory_receives_custom_onnx_model(tmp_path):
    detector, _model, calls = _detector(tmp_path)

    assert calls == [{
        'wakeword_models': [detector.model_path],
        'inference_framework': 'onnx',
    }]


@pytest.mark.parametrize('threshold', [
    True,
    0,
    -0.01,
    1.01,
    float('inf'),
    float('nan'),
    'not-a-number',
])
def test_invalid_threshold_is_rejected_before_model_loading(threshold):
    with pytest.raises(ValueError, match=r'threshold.*\(0, 1\]'):
        OpenWakeWordDetector(
            'missing.onnx',
            threshold,
            model_factory=lambda **_kwargs: _FakeModel(),
        )


def test_threshold_boundaries_are_inclusive(tmp_path):
    detector, model, _calls = _detector(
        tmp_path, scores=[0.499, 0.5, 0.8], threshold=0.5)

    score = detector.process(
        np.zeros(3 * detector.FRAME_SAMPLES, dtype=np.float32),
        sample_rate=16000,
    )

    assert score == pytest.approx(0.8)
    assert len(model.frames) == 3


def test_no_score_at_threshold_returns_none(tmp_path):
    detector, model, _calls = _detector(
        tmp_path, scores=[0.1, 0.49], threshold=0.5)

    assert detector.process(
        np.zeros(2 * detector.FRAME_SAMPLES, dtype=np.float32),
        sample_rate=16000,
    ) is None
    assert len(model.frames) == 2


def test_16khz_audio_is_buffered_into_exact_pcm16_frames(tmp_path):
    detector, model, _calls = _detector(tmp_path)
    first = np.linspace(
        -1.5, 1.5, detector.FRAME_SAMPLES - 17, dtype=np.float32)
    second = np.zeros(17, dtype=np.float32)

    assert detector.process(first[:, None], sample_rate=16000) is None
    assert model.frames == []
    assert detector.process(second, sample_rate=16000) is None

    assert len(model.frames) == 1
    frame = model.frames[0]
    assert frame.shape == (detector.FRAME_SAMPLES,)
    assert frame.dtype == np.int16
    assert frame[0] == np.iinfo(np.int16).min
    assert frame[-18] == np.iinfo(np.int16).max
    assert frame[-17:].tolist() == [0] * 17


def test_48khz_audio_produces_80ms_16khz_frames(tmp_path):
    detector, model, _calls = _detector(tmp_path)

    detector.process(
        np.zeros(4800, dtype=np.float32),
        sample_rate=48000,
    )

    # A 100-ms capture chunk becomes 1600 target-rate samples: one complete
    # 1280-sample/80-ms model frame plus 320 samples retained for the next call.
    assert len(model.frames) == 1
    assert model.frames[0].shape == (1280,)
    assert model.frames[0].dtype == np.int16

    detector.process(
        np.zeros(2880, dtype=np.float32),
        sample_rate=48000,
    )
    assert len(model.frames) == 2


def test_48khz_resampling_is_independent_of_chunk_boundaries(tmp_path):
    sample_count = 48000
    source = (
        0.6 * np.sin(
            2.0 * np.pi * 1000.0
            * np.arange(sample_count, dtype=np.float64)
            / 48000.0
        )
    ).astype(np.float32)

    whole, whole_model, _calls = _detector(tmp_path / 'whole')
    chunked, chunked_model, _calls = _detector(tmp_path / 'chunked')

    whole.process(source, sample_rate=48000)
    offsets = [0, 137, 5001, 9017, 17422, 31003, sample_count]
    for start, end in zip(offsets, offsets[1:]):
        chunked.process(source[start:end], sample_rate=48000)

    whole_pcm = np.concatenate(whole_model.frames)
    chunked_pcm = np.concatenate(chunked_model.frames)
    np.testing.assert_array_equal(chunked_pcm, whole_pcm)
    assert len(whole_model.frames) == len(chunked_model.frames) == 12


def test_reset_discards_partial_frame_and_resets_model(tmp_path):
    detector, model, _calls = _detector(tmp_path)
    half_frame = np.ones(detector.FRAME_SAMPLES // 2, dtype=np.float32)

    detector.process(half_frame, sample_rate=16000)
    detector.reset()
    detector.process(half_frame, sample_rate=16000)

    assert model.frames == []
    assert model.reset_calls == 1
    detector.process(half_frame, sample_rate=16000)
    assert len(model.frames) == 1


def test_input_rate_change_resets_all_streaming_state(tmp_path):
    detector, model, _calls = _detector(tmp_path)
    detector.process(np.ones(640, dtype=np.float32), sample_rate=16000)

    detector.process(np.zeros(3840, dtype=np.float32), sample_rate=48000)

    assert model.reset_calls == 1
    assert len(model.frames) == 1
    # No 16-kHz samples from the previous microphone leaked into this frame.
    assert np.all(model.frames[0] == 0)


@pytest.mark.parametrize('sample_rate', [0, 8000, 44100, 96000, '16000'])
def test_unsupported_input_rate_is_rejected(tmp_path, sample_rate):
    detector, _model, _calls = _detector(tmp_path)

    with pytest.raises(ValueError, match='unsupported wake-word input rate'):
        detector.process(
            np.zeros(detector.FRAME_SAMPLES, dtype=np.float32),
            sample_rate=sample_rate,
        )


def test_non_mono_or_non_finite_audio_is_rejected(tmp_path):
    detector, _model, _calls = _detector(tmp_path)

    with pytest.raises(ValueError, match='mono'):
        detector.process(np.zeros((1280, 2)), sample_rate=16000)
    with pytest.raises(ValueError, match='finite'):
        detector.process(np.array([0.0, np.nan]), sample_rate=16000)
    with pytest.raises(TypeError, match='floating-point'):
        detector.process(np.zeros(1280, dtype=np.int16), sample_rate=16000)


@pytest.mark.parametrize('predictions', [
    {},
    {'robot': 'not-a-number'},
    {'robot': -0.1},
    {'robot': 1.1},
    {'robot': float('nan')},
])
def test_invalid_model_scores_fail_closed(tmp_path, predictions):
    detector, model, _calls = _detector(tmp_path)
    model.predict = lambda _frame: predictions

    with pytest.raises(RuntimeError, match='openWakeWord'):
        detector.process(
            np.zeros(detector.FRAME_SAMPLES, dtype=np.float32),
            sample_rate=16000,
        )


def test_missing_or_non_onnx_model_is_rejected(tmp_path):
    with pytest.raises(FileNotFoundError, match='does not exist'):
        OpenWakeWordDetector(
            tmp_path / 'missing.onnx',
            0.5,
            model_factory=lambda **_kwargs: _FakeModel(),
        )

    wrong_format = tmp_path / 'robot.tflite'
    wrong_format.write_bytes(b'fake model')
    with pytest.raises(ValueError, match='ONNX'):
        OpenWakeWordDetector(
            wrong_format,
            0.5,
            model_factory=lambda **_kwargs: _FakeModel(),
        )
