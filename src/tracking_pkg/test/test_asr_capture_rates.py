#!/usr/bin/env python3
"""Tests for native-rate microphone input passed to faster-whisper."""

import io
import os
import sys
import wave
from types import SimpleNamespace

import numpy as np


_PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_PKG, 'src', 'interfaces'))

from asr_node import ASRNode  # noqa: E402


class _FakeModel:
    def __init__(self):
        self.audio = None

    def transcribe(self, audio, **_kwargs):
        self.audio = audio
        info = SimpleNamespace(language='en', language_probability=1.0)
        return iter([SimpleNamespace(text='robot stop')]), info


def _node_for_transcribe(capture_rate, whisper_rate=16000):
    node = ASRNode.__new__(ASRNode)
    node._model = _FakeModel()
    node.capture_sample_rate = capture_rate
    node.sample_rate = whisper_rate
    node.language = 'en'
    node.initial_prompt = None
    node.get_logger = lambda: SimpleNamespace(
        info=lambda *_args: None,
        error=lambda *_args: None,
    )
    return node


def test_native_whisper_rate_stays_numpy():
    node = _node_for_transcribe(16000)
    audio = np.zeros(1600, dtype=np.float32)

    assert node._transcribe(audio) == 'robot stop'
    assert node._model.audio is audio


def test_jieli_rate_is_wrapped_as_48khz_wav_for_decoder_resampling():
    node = _node_for_transcribe(48000)
    audio = np.linspace(-0.5, 0.5, 4800, dtype=np.float32)

    assert node._transcribe(audio) == 'robot stop'
    assert isinstance(node._model.audio, io.BytesIO)

    with wave.open(node._model.audio, 'rb') as wav_file:
        assert wav_file.getnchannels() == 1
        assert wav_file.getsampwidth() == 2
        assert wav_file.getframerate() == 48000
        assert wav_file.getnframes() == len(audio)
