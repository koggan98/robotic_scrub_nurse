#!/usr/bin/env python3
"""
Streaming openWakeWord adapter for normalized microphone audio.

The ROS ASR node captures mono ``float32`` audio at either 16 kHz (Samson) or
48 kHz (Jieli). openWakeWord expects signed 16-bit, 16-kHz PCM in multiples of
80 ms. This module bridges those contracts without importing openWakeWord until
an actual detector is constructed, keeping the pure ASR tests dependency-free.
"""

import math
import os
from collections.abc import Mapping
from typing import Callable, Optional

import numpy as np


class WakeWordDependencyError(RuntimeError):
    """Raised when an optional wake-word runtime dependency is unavailable."""


def _default_model_factory(**kwargs):
    """Construct openWakeWord lazily so importing this helper stays cheap."""
    try:
        from openwakeword.model import Model
    except ImportError as exc:
        raise WakeWordDependencyError(
            'openwakeword is not installed; install openwakeword==0.6.0'
        ) from exc
    return Model(**kwargs)


class _StreamingRateConverter:
    """
    Convert a continuous mono stream from 16/48 kHz to 16 kHz.

    The 48-kHz path keeps both the FIR filter state and the decimation phase.
    Therefore splitting the same stream into different capture chunks produces
    the same output samples, with no gaps or duplicated samples at boundaries.
    """

    OUTPUT_RATE = 16000
    SUPPORTED_INPUT_RATES = (16000, 48000)
    _DOWNSAMPLE_FACTOR = 3
    _FILTER_TAPS = 63
    # Leave a transition band below the 8-kHz output Nyquist frequency.
    _FILTER_CUTOFF_HZ = 7200.0

    def __init__(self, input_rate: int):
        if input_rate not in self.SUPPORTED_INPUT_RATES:
            rates = ', '.join(str(rate) for rate in self.SUPPORTED_INPUT_RATES)
            raise ValueError(
                f'unsupported wake-word input rate {input_rate}; '
                f'expected one of: {rates}')

        self.input_rate = input_rate
        self._filter = None
        self._filter_state = None
        self._input_phase = 0

        if input_rate == 48000:
            try:
                from scipy.signal import firwin, lfilter
            except ImportError as exc:
                raise WakeWordDependencyError(
                    'scipy is required for 48-kHz wake-word audio'
                ) from exc

            self._lfilter = lfilter
            self._filter = firwin(
                self._FILTER_TAPS,
                cutoff=self._FILTER_CUTOFF_HZ,
                fs=input_rate,
                window=('kaiser', 5.0),
            )
            self._filter_state = np.zeros(
                len(self._filter) - 1, dtype=np.float64)

    def process(self, samples: np.ndarray) -> np.ndarray:
        """Return all currently available 16-kHz normalized float samples."""
        if self.input_rate == self.OUTPUT_RATE:
            return samples

        filtered, self._filter_state = self._lfilter(
            self._filter,
            [1.0],
            samples,
            zi=self._filter_state,
        )

        # Select source positions whose absolute stream index is divisible by
        # three. Keeping the phase makes arbitrary chunk boundaries transparent.
        first_output = (-self._input_phase) % self._DOWNSAMPLE_FACTOR
        output = filtered[first_output::self._DOWNSAMPLE_FACTOR]
        self._input_phase = (
            self._input_phase + len(samples)
        ) % self._DOWNSAMPLE_FACTOR
        return output


class OpenWakeWordDetector:
    """
    Feed streaming microphone chunks to one custom openWakeWord model.

    Parameters
    ----------
    model_path
        Existing custom ONNX wake-word model.
    threshold
        Inclusive positive-detection threshold in ``(0, 1]``.
    model_factory
        Optional test seam accepting the same keyword arguments as
        ``openwakeword.model.Model``.

    ``process`` returns the highest score at or above ``threshold`` observed in
    the newly completed 80-ms frames, or ``None`` when there was no activation.

    """

    SAMPLE_RATE = 16000
    FRAME_DURATION_SECONDS = 0.08
    FRAME_SAMPLES = 1280
    SUPPORTED_INPUT_RATES = _StreamingRateConverter.SUPPORTED_INPUT_RATES

    def __init__(
        self,
        model_path,
        threshold,
        model_factory: Optional[Callable] = None,
    ):
        if isinstance(threshold, bool):
            raise ValueError('wake-word threshold must be a number in (0, 1]')
        try:
            normalized_threshold = float(threshold)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                'wake-word threshold must be a number in (0, 1]'
            ) from exc
        if (
            not math.isfinite(normalized_threshold)
            or normalized_threshold <= 0.0
            or normalized_threshold > 1.0
        ):
            raise ValueError('wake-word threshold must be in (0, 1]')

        try:
            normalized_path = os.path.abspath(os.fspath(model_path))
        except TypeError as exc:
            raise ValueError(
                'wake-word model path must be a filesystem path'
            ) from exc
        if not normalized_path.lower().endswith('.onnx'):
            raise ValueError('wake-word model must be an ONNX file')
        if not os.path.isfile(normalized_path):
            raise FileNotFoundError(
                f'wake-word model does not exist: {normalized_path}')

        self.model_path = normalized_path
        self.threshold = normalized_threshold
        factory = model_factory or _default_model_factory
        self._model = factory(
            wakeword_models=[self.model_path],
            inference_framework='onnx',
        )

        self._input_rate = None
        self._rate_converter = None
        self._frame_buffer = np.empty(0, dtype=np.float64)

    @staticmethod
    def _mono_float_samples(audio_chunk) -> np.ndarray:
        """Validate and flatten normalized mono audio from sounddevice."""
        samples = np.asarray(audio_chunk)
        if samples.ndim == 2 and samples.shape[1] == 1:
            samples = samples[:, 0]
        elif samples.ndim != 1:
            raise ValueError(
                'wake-word audio must be mono with shape (samples,) or '
                '(samples, 1)')
        if not np.issubdtype(samples.dtype, np.floating):
            raise TypeError(
                'wake-word audio must contain normalized floating-point samples')

        samples = samples.astype(np.float64, copy=False)
        if not np.all(np.isfinite(samples)):
            raise ValueError('wake-word audio samples must be finite')
        return samples

    @staticmethod
    def _pcm16(samples: np.ndarray) -> np.ndarray:
        """Map normalized floats to the complete signed PCM16 range."""
        clipped = np.clip(samples, -1.0, 1.0)
        scaled = np.where(
            clipped >= 0.0,
            clipped * np.iinfo(np.int16).max,
            clipped * -np.iinfo(np.int16).min,
        )
        return scaled.astype(np.int16)

    @staticmethod
    def _highest_score(predictions) -> float:
        if not isinstance(predictions, Mapping) or not predictions:
            raise RuntimeError(
                'openWakeWord predict() must return a non-empty score mapping')

        scores = []
        for value in predictions.values():
            try:
                score = float(value)
            except (TypeError, ValueError) as exc:
                raise RuntimeError(
                    'openWakeWord returned a non-numeric score') from exc
            if not math.isfinite(score) or score < 0.0 or score > 1.0:
                raise RuntimeError(
                    f'openWakeWord returned an invalid score: {score}')
            scores.append(score)
        return max(scores)

    def _configure_input_rate(self, sample_rate):
        if sample_rate not in self.SUPPORTED_INPUT_RATES:
            rates = ', '.join(str(rate) for rate in self.SUPPORTED_INPUT_RATES)
            raise ValueError(
                f'unsupported wake-word input rate {sample_rate}; '
                f'expected one of: {rates}')

        if self._input_rate is not None and sample_rate != self._input_rate:
            # A hot-swapped microphone starts a new stream. Discard the old
            # resampler/model history so it cannot contribute to a later hit.
            self.reset()

        if self._input_rate is None:
            self._input_rate = int(sample_rate)
            self._rate_converter = _StreamingRateConverter(self._input_rate)

    def process(self, audio_chunk, sample_rate) -> Optional[float]:
        """Process one mono capture chunk and return a hit score or ``None``."""
        self._configure_input_rate(sample_rate)
        samples = self._mono_float_samples(audio_chunk)
        if samples.size == 0:
            return None

        converted = self._rate_converter.process(samples)
        if self._frame_buffer.size:
            converted = np.concatenate((self._frame_buffer, converted))

        complete_samples = (
            converted.size // self.FRAME_SAMPLES
        ) * self.FRAME_SAMPLES
        strongest_hit = None
        for offset in range(0, complete_samples, self.FRAME_SAMPLES):
            frame = self._pcm16(
                converted[offset:offset + self.FRAME_SAMPLES])
            score = self._highest_score(self._model.predict(frame))
            if score >= self.threshold:
                strongest_hit = (
                    score if strongest_hit is None
                    else max(strongest_hit, score)
                )

        self._frame_buffer = converted[complete_samples:].copy()
        return strongest_hit

    def reset(self):
        """Clear model, resampler, and incomplete-frame history."""
        self._input_rate = None
        self._rate_converter = None
        self._frame_buffer = np.empty(0, dtype=np.float64)
        model_reset = getattr(self._model, 'reset', None)
        if callable(model_reset):
            model_reset()
