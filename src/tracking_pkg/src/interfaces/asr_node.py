#!/usr/bin/env python3
"""
Capture microphone speech and publish commands transcribed by Whisper.

In the active Jetson setup, a lightweight openWakeWord model runs before the
Whisper queue. Ordinary room conversation is therefore discarded without an
expensive transcription. Acoustic hits are verified as an exact wake token,
then one command is captured and published on ``/user_speech``.

The runtime dependencies are faster-whisper, sounddevice, NumPy, SciPy, and
``openwakeword==0.6.0``.
"""

from collections import deque
from dataclasses import dataclass
import io
import json
import math
import os
import queue
import time
import wave
from threading import Lock, Thread

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from wake_word import (
    correct_addressed_command,
    is_exact_wake_utterance,
    plan_wake_segment,
)


@dataclass(frozen=True)
class AudioQueueItem:
    """One transcription candidate tied to an acoustic wake session."""

    kind: str
    generation: int
    audio: np.ndarray
    started_at: float
    deadline_reserved: bool
    # Monotonic time at which the VAD closed this segment (utterance end / cut).
    # Used only for the /asr_timing latency diagnostic.
    speech_end_at: float = 0.0


class AudioDeviceSelectionError(RuntimeError):
    """Raised when automatic input selection is ambiguous or misconfigured."""


def select_supported_audio_device(devices, candidates, sample_rates):
    """
    Return the single supported input as ``(index, name, rate, match)``.

    ``None`` means that no configured microphone is currently present. Output-only
    devices are ignored. More than one match is rejected so the robot never picks
    an arbitrary microphone when the hardware setup is ambiguous.
    """
    if len(candidates) != len(sample_rates):
        raise AudioDeviceSelectionError(
            'audio device candidates and sample rates must have equal length')

    matches = {}
    for candidate, sample_rate in zip(candidates, sample_rates):
        if not candidate or int(sample_rate) <= 0:
            raise AudioDeviceSelectionError(
                'audio device candidates must have names and positive rates')

        candidate_folded = str(candidate).casefold()
        for index, device in enumerate(devices):
            if int(device['max_input_channels']) <= 0:
                continue
            device_name = str(device['name'])
            if candidate_folded in device_name.casefold():
                matches[index] = (
                    index, device_name, int(sample_rate), str(candidate))

    if not matches:
        return None
    if len(matches) > 1:
        names = ', '.join(match[1] for match in matches.values())
        raise AudioDeviceSelectionError(
            f'multiple supported microphones detected: {names}')
    return next(iter(matches.values()))


def audio_capture_block_size(sample_rate, chunk_duration):
    """Return the number of samples in one capture chunk."""
    return int(sample_rate * chunk_duration)


def _positive_finite_parameter(name, value):
    """Return one positive finite floating-point ROS parameter."""
    try:
        normalized = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f'{name} must be a positive finite number') from exc
    if not math.isfinite(normalized) or normalized <= 0.0:
        raise ValueError(f'{name} must be a positive finite number')
    return normalized


class ASRNode(Node):
    def __init__(self):
        super().__init__('asr_node')

        # Parameters
        self.declare_parameter('whisper_model', 'base')
        self.declare_parameter('language', 'de')
        self.declare_parameter('sample_rate', 16000)
        # Some direct-ALSA microphones expose only one hardware rate. In
        # particular, the Jieli USB receiver is 48 kHz-only while Whisper is
        # 16 kHz. A non-matching capture rate is resampled by faster-whisper's
        # normal file decoder before transcription.
        self.declare_parameter('capture_sample_rate', 0)
        self.declare_parameter('silence_threshold_seconds', 0.8)
        self.declare_parameter('energy_threshold', 0.015)
        self.declare_parameter('min_speech_seconds', 0.5)
        # Hard bound for one deliberate wake/command segment. Idle audio keeps
        # only the 1.5-s pre-roll, but an active segment also needs a cap so a
        # stuck VAD or continuous tone cannot grow memory or hold the FSM open.
        self.declare_parameter('max_speech_seconds', 8.0)
        self.declare_parameter('device_index', -1)
        # Select the input device by NAME substring (passed straight to sounddevice),
        # e.g. 'Samson'. Preferred over device_index: robust to index changes and works
        # when PulseAudio exposes no capture source (direct ALSA). '' -> use device_index.
        self.declare_parameter('audio_device', '')
        # When neither audio_device nor device_index is explicit, select the one
        # connected supported input. Rates are paired by list position.
        self.declare_parameter(
            'audio_device_candidates', ['Samson', 'USB Composite Device'])
        self.declare_parameter(
            'audio_device_candidate_rates', [16000, 48000])
        self.declare_parameter('audio_device_retry_seconds', 5.0)
        # Compute device for faster-whisper: 'cuda' (Spark default) or 'cpu'.
        self.declare_parameter('device', 'cuda')
        # CTranslate2 compute type; empty -> auto (float16 on GPU, int8 on CPU).
        self.declare_parameter('compute_type', '')
        # >0 caps CTranslate2's CPU thread count. On the Jetson Whisper runs on CPU
        # (no CUDA ctranslate2 build), so leave cores free for sshd/other nodes.
        self.declare_parameter('cpu_threads', 0)
        # Only utterances addressed to the robot become commands: the transcript
        # must START with one of these (fillers like "hey" in front are fine).
        # The wake word is stripped before publishing. Empty list = no gate.
        # NOTE: direct injection via `ros2 topic pub /user_speech ...` bypasses
        # the microphone and therefore needs no wake word — by design.
        self.declare_parameter('wake_words', [
            'robot', 'robo', 'rob', 'robi', 'robbie', 'robert'])
        # Fuzzy acceptance for Whisper re-spellings ("Roby", "Robots").
        self.declare_parameter('wake_word_fuzzy', 0.75)
        # Optional acoustic pre-filter. The active distributed Jetson launch
        # enables it and points at the packaged custom "Robot" ONNX model.
        # Other/legacy launches stay transcription-first unless they explicitly
        # opt in. An enabled but unloadable detector is a startup error: silently
        # falling back would recreate the background-speech queue this gate fixes.
        self.declare_parameter('audio_wake_enabled', False)
        self.declare_parameter('audio_wake_model_path', '')
        self.declare_parameter('audio_wake_threshold', 0.5)
        # True means the wake candidate must transcribe to the wake token alone.
        # "Robot needle holder" is rejected; say "Robot", pause, then command.
        self.declare_parameter('wake_require_separate_command', False)
        # Legacy text-gate policy: saying "robot" alone arms a second segment,
        # while a one-breath command remains accepted. The active acoustic path
        # additionally sets wake_require_separate_command, which intentionally
        # rejects that one-breath form.
        self.declare_parameter('two_stage_wake', True)
        # After the wake word, how long to wait for the command to START before
        # disarming back to idle.
        self.declare_parameter('command_window_sec', 6.0)
        # faster-whisper no_speech probability above which a segment is treated
        # as silence and skipped fast (keeps noise from blocking the pipeline).
        self.declare_parameter('no_speech_threshold', 0.6)
        # Domain bias for Whisper's decoder: fed as initial_prompt, it pulls
        # the transcription toward this vocabulary — "end surgery" instead of
        # "and surgery", "finish" instead of "Finnish". Practically free
        # compared to a bigger model. Empty string disables.
        self.declare_parameter('initial_prompt', (
            'Robot commands in an operating room: robot, end surgery, '
            'finish surgery, start surgery, count the instruments, '
            'needle holder, forceps, tweezers, scissors, retractor, awl, '
            'pick up the awl, return the awl, hammer, put it back, wrong tool, '
            'release, stop.'))

        self.whisper_model_size = self.get_parameter('whisper_model').value
        self.language = self.get_parameter('language').value or None
        self.sample_rate = int(self.get_parameter('sample_rate').value)
        capture_sample_rate = int(
            self.get_parameter('capture_sample_rate').value)
        self.capture_sample_rate = capture_sample_rate or self.sample_rate
        if self.sample_rate <= 0 or self.capture_sample_rate <= 0:
            raise ValueError('sample rates must be positive')
        self.silence_threshold = float(self.get_parameter('silence_threshold_seconds').value)
        self.energy_threshold = float(self.get_parameter('energy_threshold').value)
        self.min_speech_seconds = float(self.get_parameter('min_speech_seconds').value)
        self.max_speech_seconds = _positive_finite_parameter(
            'max_speech_seconds',
            self.get_parameter('max_speech_seconds').value,
        )
        device_idx = int(self.get_parameter('device_index').value)
        audio_device = self.get_parameter('audio_device').value
        self.audio_device_candidates = list(
            self.get_parameter('audio_device_candidates').value or [])
        self.audio_device_candidate_rates = [
            int(rate) for rate in
            (self.get_parameter('audio_device_candidate_rates').value or [])
        ]
        self.audio_device_retry_seconds = float(
            self.get_parameter('audio_device_retry_seconds').value)
        if self.audio_device_retry_seconds <= 0:
            raise ValueError('audio_device_retry_seconds must be positive')
        if len(self.audio_device_candidates) != len(
                self.audio_device_candidate_rates):
            raise ValueError(
                'audio_device_candidates and audio_device_candidate_rates '
                'must have equal length')

        # Explicit name/index selection stays available for diagnostics and
        # backwards compatibility. Otherwise, configured candidates are scanned.
        self.device_index = audio_device if audio_device else (
            None if device_idx < 0 else device_idx)
        self._auto_select_audio = (
            self.device_index is None and bool(self.audio_device_candidates))
        self._last_audio_detection_issue = None
        self.asr_device = self.get_parameter('device').value or 'cuda'
        compute_type = self.get_parameter('compute_type').value
        self.cpu_threads = int(self.get_parameter('cpu_threads').value)
        self.compute_type = compute_type or (
            'float16' if str(self.asr_device).startswith('cuda') else 'int8')
        self.wake_words = list(self.get_parameter('wake_words').value or [])
        self.wake_word_fuzzy = float(self.get_parameter('wake_word_fuzzy').value)
        self.audio_wake_enabled = bool(
            self.get_parameter('audio_wake_enabled').value)
        self.audio_wake_model_path = str(
            self.get_parameter('audio_wake_model_path').value or '')
        self.audio_wake_threshold = float(
            self.get_parameter('audio_wake_threshold').value)
        if not 0.0 < self.audio_wake_threshold <= 1.0:
            raise ValueError('audio_wake_threshold must be in (0, 1]')
        self.wake_require_separate_command = bool(
            self.get_parameter('wake_require_separate_command').value)
        self.initial_prompt = self.get_parameter('initial_prompt').value or None
        self.two_stage_wake = bool(self.get_parameter('two_stage_wake').value)
        self.command_window_sec = _positive_finite_parameter(
            'command_window_sec',
            self.get_parameter('command_window_sec').value,
        )
        self.no_speech_threshold = float(
            self.get_parameter('no_speech_threshold').value)

        # Publishers
        self.publisher = self.create_publisher(String, 'user_speech', 10)
        # Diagnostic-only: per accepted command, a JSON String with monotonic
        # timestamps (segment-cut vs publish) so an external logger can measure
        # transcription latency. No effect on the command path.
        self._timing_pub = self.create_publisher(String, 'asr_timing', 10)
        # HRI status for the visual display: 'listening' (armed, waiting for the
        # command -> amber "speak now") or '' (idle). A pure feedback channel.
        # Latched: the HRI display (which boots faster than the Whisper model
        # loads) still gets the last status — including the initial 'ready' —
        # whenever it (re)subscribes.
        self.status_publisher = self.create_publisher(
            String, 'asr_status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        # Wake FSM state. With the acoustic filter there is an intermediate
        # verification phase; without it the legacy text-only flow uses the
        # original wait_wake/command pair.
        # ``resetting`` is a short internal barrier: capture cannot run the
        # detector between closing a session and clearing the model's history.
        self._phase = 'wait_wake'  # wait_wake | verify_wake | command | resetting
        self._command_deadline = 0.0
        self._wake_generation = 0
        self._wake_state_lock = Lock()
        self._detector_lock = Lock()
        self._active_segment_generation = None
        self._active_segment_started_at = None
        # A command may finish and enter the queue in the narrow interval
        # between Queue.get() timing out and the worker checking the session
        # deadline. Reserving it under the FSM lock prevents that check from
        # invalidating an otherwise in-time command.
        self._pending_in_time_commands = 0
        self._wake_detector = None

        # Audio state. Capture runs continuously on one thread and hands whole
        # speech segments to a transcription thread through this queue, so the
        # microphone is NEVER closed while Whisper is busy — speech spoken during
        # a transcription is captured instead of lost. The queue is bounded: if
        # transcription lags (loaded Jetson), the oldest stale segment is dropped
        # rather than letting a backlog grow without limit.
        self._max_queued_segments = 8
        self._audio_queue = queue.Queue()
        self._model = None
        self._running = True
        self._fatal_error = None

        # Load the small wake model synchronously so a missing dependency/model
        # fails the process before it can claim to be ready. Whisper is still
        # loaded lazily on the listening thread after a microphone is available.
        if self.audio_wake_enabled:
            self._wake_detector = self._create_wake_detector()

        capture_rate_description = (
            'auto' if self._auto_select_audio
            else f'{self.capture_sample_rate}Hz')
        self.get_logger().info(
            f'ASRNode starting (model={self.whisper_model_size}, '
            f'device={self.asr_device}/{self.compute_type}, '
            f'lang={self.language or "auto"}, '
            f'capture_rate={capture_rate_description}, '
            f'whisper_rate={self.sample_rate}Hz, '
            f'silence={self.silence_threshold}s, energy={self.energy_threshold}, '
            f'acoustic_wake={self.audio_wake_enabled})'
        )

        # Start audio + transcription threads
        self._listen_thread = Thread(target=self._listen_loop, daemon=True)
        self._listen_thread.start()

    def _create_wake_detector(self):
        """Create the acoustic detector or fail closed during node startup."""
        if not self.audio_wake_model_path:
            message = (
                'audio_wake_enabled is true but audio_wake_model_path is empty')
            self.get_logger().error(message)
            raise RuntimeError(message)
        if not os.path.isfile(self.audio_wake_model_path):
            message = (
                'Acoustic wake model does not exist: '
                f'{self.audio_wake_model_path}')
            self.get_logger().error(message)
            raise RuntimeError(message)
        try:
            from openwakeword_detector import OpenWakeWordDetector
            detector = OpenWakeWordDetector(
                self.audio_wake_model_path,
                self.audio_wake_threshold,
            )
        except Exception as exc:
            message = (
                'Failed to initialize acoustic wake detector '
                f'({self.audio_wake_model_path}): {exc}')
            self.get_logger().error(message)
            raise RuntimeError(message) from exc
        self.get_logger().info(
            'Acoustic wake detector ready '
            f'(model={self.audio_wake_model_path}, '
            f'threshold={self.audio_wake_threshold:.2f})')
        return detector

    def _load_model(self):
        """
        Load the faster-whisper model when the first audio input is ready.

        Defaults to CUDA on the Spark; degrades to CPU/int8 if the GPU build of
        CTranslate2 is unavailable so ASR still works without a usable GPU.
        """
        try:
            from faster_whisper import WhisperModel
        except ImportError:
            self.get_logger().error(
                'faster-whisper not installed. Run: pip install faster-whisper'
            )
            return False
        try:
            self.get_logger().info(
                f'Loading faster-whisper model "{self.whisper_model_size}" '
                f'on {self.asr_device} ({self.compute_type})...'
            )
            self._model = WhisperModel(
                self.whisper_model_size,
                device=self.asr_device,
                compute_type=self.compute_type,
                cpu_threads=self.cpu_threads,
            )
            self.get_logger().info('Whisper model loaded successfully')
            return True
        except Exception as e:
            if str(self.asr_device).startswith('cuda'):
                self.get_logger().warn(
                    f'CUDA whisper load failed ({e}); falling back to CPU/int8'
                )
                self.asr_device = 'cpu'
                self.compute_type = 'int8'
                try:
                    self._model = WhisperModel(
                        self.whisper_model_size,
                        device=self.asr_device,
                        compute_type=self.compute_type,
                        cpu_threads=self.cpu_threads,
                    )
                    self.get_logger().info('Whisper model loaded successfully (CPU)')
                    return True
                except Exception as e2:
                    self.get_logger().error(f'Failed to load whisper model: {e2}')
                    return False
            self.get_logger().error(f'Failed to load whisper model: {e}')
            return False

    def _listen_loop(self):
        """
        Set up audio and run the continuous capture producer.

        A separate worker thread transcribes queued segments and runs the wake
        FSM, so the microphone stays open while Whisper is busy.
        """
        try:
            import sounddevice as sd
        except ImportError:
            self.get_logger().error(
                'sounddevice not installed. Run: pip install sounddevice'
            )
            return

        # Find a usable microphone before spending CPU/RAM on the Whisper model.
        if not self._wait_for_audio_input(sd):
            return

        if not self._load_model():
            return

        # Model loaded and a mic is open: the node is now genuinely listening.
        # This flips the HRI display from "starting up" to ready.
        self._publish_status('ready')

        # Consumer: transcribe queued segments + run the wake FSM.
        self._transcribe_thread = Thread(
            target=self._transcribe_worker, daemon=True)
        self._transcribe_thread.start()

        # Producer: keep the mic open and push speech segments to the queue.
        self._capture_loop(sd)

    def _capture_loop(self, sd):
        """
        Keep the microphone open and enqueue speech segments.

        Transcription runs on the worker thread and never blocks this producer.
        """
        chunk_duration = 0.1  # 100ms chunks
        _no_mic_warned = False
        while self._running and rclpy.ok():
            try:
                self._stream_and_segment(sd, chunk_duration)
                _no_mic_warned = False
            except sd.PortAudioError as e:
                self._reset_acoustic_wake_runtime('audio device disconnected')
                if not self._running:
                    return
                if self._auto_select_audio:
                    self.get_logger().warn(
                        f'Audio device lost ({e}); rescanning supported microphones.'
                    )
                    self.device_index = None
                    if not self._wait_for_audio_input(sd, refresh=True):
                        return
                    continue

                err_str = str(e)
                if 'Input/output error' in err_str or 'ALSA error -5' in err_str:
                    if not _no_mic_warned:
                        _no_mic_warned = True
                        self.get_logger().warn(
                            'No microphone available (ALSA I/O error). '
                            'Inject speech via: ros2 topic pub --once /user_speech '
                            'std_msgs/msg/String "{data: \'your command\'}". '
                            'Retrying every 60 s.'
                        )
                    time.sleep(60.0)
                else:
                    self.get_logger().error(f'Audio device error: {e}')
                    time.sleep(2.0)
            except Exception as e:
                self._reset_acoustic_wake_runtime('capture loop error')
                self.get_logger().error(f'Capture loop error: {e}')
                if self._running:
                    time.sleep(1.0)

    def _refresh_portaudio_devices(self, sd):
        """Refresh PortAudio's frozen device list while no stream is open."""
        terminate = getattr(sd, '_terminate', None)
        initialize = getattr(sd, '_initialize', None)
        if not callable(terminate) or not callable(initialize):
            return
        terminate()
        initialize()

    def _apply_audio_selection(self, selection):
        """Apply an automatic selection to the next capture stream."""
        index, _name, sample_rate, _candidate = selection
        changed = (
            self.device_index is not None and
            (self.device_index != index or
             self.capture_sample_rate != sample_rate))
        self.device_index = index
        self.capture_sample_rate = sample_rate
        if changed:
            self._reset_acoustic_wake_runtime(
                'microphone selection changed')

    def _wait_for_audio_input(self, sd, refresh=False):
        """Wait until exactly one configured input is connected."""
        if not self._auto_select_audio:
            devices = sd.query_devices()
            self.get_logger().info(f'Audio devices:\n{devices}')
            if self.device_index is None:
                default_input = sd.query_devices(kind='input')['name']
                self.get_logger().info(
                    f'Using default input device: {default_input}')
            else:
                self.get_logger().info(
                    f'Using requested input device matching: '
                    f'{self.device_index!r} at {self.capture_sample_rate}Hz')
            return True

        should_refresh = refresh
        while self._running and rclpy.ok():
            try:
                if should_refresh:
                    self._refresh_portaudio_devices(sd)
                devices = sd.query_devices()
                selection = select_supported_audio_device(
                    devices,
                    self.audio_device_candidates,
                    self.audio_device_candidate_rates,
                )
                if selection is not None:
                    index, name, sample_rate, candidate = selection
                    self._apply_audio_selection(selection)
                    self._last_audio_detection_issue = None
                    self.get_logger().info(
                        f'Automatically selected microphone: {name} '
                        f'(match={candidate!r}, index={index}, '
                        f'capture_rate={sample_rate}Hz)')
                    return True
                issue = (
                    'No supported microphone detected; connect Samson Q2U or '
                    'the Jieli USB receiver. Retrying every '
                    f'{self.audio_device_retry_seconds:g} s.')
                log = self.get_logger().warn
            except AudioDeviceSelectionError as exc:
                issue = f'Automatic microphone selection rejected: {exc}'
                log = self.get_logger().error
            except Exception as exc:
                issue = f'Failed to query audio devices: {exc}'
                log = self.get_logger().error

            if issue != self._last_audio_detection_issue:
                log(issue)
                self._last_audio_detection_issue = issue
            time.sleep(self.audio_device_retry_seconds)
            # PortAudio freezes device indices at initialization. Refresh before
            # every later scan so a newly connected USB microphone becomes visible.
            should_refresh = True
        return False

    def _stream_and_segment(self, sd, chunk_duration):
        """
        Hold one input stream and route complete energy-VAD segments.

        openWakeWord sees every raw chunk while the node is idle. The energy
        segmenter remains responsible for defining the audio sent to Whisper,
        but segments without an acoustic hit are discarded before the queue.
        A small rolling pre-roll retains the beginning of a quiet wake word when
        the acoustic model fires before/without the energy threshold.
        """
        speech_buffer = []
        is_speaking = False
        silence_start = None
        segment_started_at = None
        buffer_started_at = None
        segment_wake_generation = None
        pre_roll = deque(
            maxlen=max(1, int(round(1.5 / float(chunk_duration)))))

        # Recomputed per stream so a Samson/Jieli hot-swap also updates the
        # 100 ms block size from 1600 to 4800 samples (or vice versa).
        chunk_samples = audio_capture_block_size(
            self.capture_sample_rate, chunk_duration)

        self.get_logger().info('Listening for speech...')
        with sd.InputStream(
            samplerate=self.capture_sample_rate,
            channels=1,
            dtype='float32',
            blocksize=chunk_samples,
            device=self.device_index,
        ) as stream:
            while self._running and rclpy.ok():
                audio_chunk, overflowed = stream.read(chunk_samples)
                if overflowed:
                    self.get_logger().debug('Audio overflow (dropped frames)')

                now = time.monotonic()
                chunk_copy = audio_chunk.copy()
                pre_roll.append(chunk_copy)
                rms = np.sqrt(np.mean(audio_chunk ** 2))
                if rms >= self.energy_threshold:
                    if not is_speaking:
                        is_speaking = True
                        segment_started_at = now
                        self._note_active_segment_started(segment_started_at)
                        self.get_logger().debug('Speech started')
                    if self._should_buffer_segment(
                            segment_wake_generation):
                        if buffer_started_at is None:
                            buffer_started_at = segment_started_at
                        speech_buffer.append(chunk_copy)
                    silence_start = None
                elif is_speaking:
                    if self._should_buffer_segment(
                            segment_wake_generation):
                        if buffer_started_at is None:
                            buffer_started_at = segment_started_at
                        speech_buffer.append(chunk_copy)
                    if silence_start is None:
                        silence_start = now

                wake_score = self._process_acoustic_wake(audio_chunk)
                if wake_score is not None:
                    generation = self._begin_wake_verification(wake_score)
                    if generation is not None:
                        segment_wake_generation = generation
                        # Always replace the idle VAD history with the bounded
                        # pre-roll. Thus quiet onsets are retained and a long
                        # background utterance never becomes a long candidate.
                        speech_buffer = list(pre_roll)
                        buffer_started_at = (
                            now - (len(pre_roll) - 1) * chunk_duration)
                        segment_started_at = buffer_started_at
                        if not is_speaking:
                            is_speaking = True
                            silence_start = now

                silence_complete = (
                    is_speaking and silence_start is not None and
                    now - silence_start >= self.silence_threshold)
                segment_limit_reached = (
                    is_speaking and buffer_started_at is not None and
                    now - buffer_started_at >= self.max_speech_seconds)
                if silence_complete or segment_limit_reached:
                    if segment_limit_reached and not silence_complete:
                        self.get_logger().warn(
                            'Maximum speech segment duration reached; '
                            'closing the current bounded segment.')
                    # Segment complete: hand it off and keep listening.
                    self._enqueue_segment(
                        speech_buffer,
                        wake_generation=segment_wake_generation,
                        segment_started_at=segment_started_at,
                    )
                    self._note_active_segment_finished(segment_started_at)
                    speech_buffer = []
                    is_speaking = False
                    silence_start = None
                    segment_started_at = None
                    buffer_started_at = None
                    segment_wake_generation = None

    def _should_buffer_segment(self, wake_generation):
        """Keep full audio only for a wake candidate or active wake session."""
        if not getattr(self, 'audio_wake_enabled', False):
            return True
        if wake_generation is not None:
            return True
        with self._wake_state_lock:
            return self._phase in ('verify_wake', 'command')

    def _process_acoustic_wake(self, audio_chunk):
        """Return a detector score only while waiting for a wake word."""
        if not getattr(self, 'audio_wake_enabled', False):
            return None
        with self._wake_state_lock:
            if self._phase != 'wait_wake':
                return None
        try:
            with self._detector_lock:
                return self._wake_detector.process(
                    np.asarray(audio_chunk).reshape(-1),
                    self.capture_sample_rate,
                )
        except Exception as exc:
            # Runtime detector failure is also fail-closed. Continuing with the
            # transcription-first path would silently restore the original bug.
            self._stop_for_acoustic_failure(
                f'Acoustic wake detector failed; ASR is stopping: {exc}')
            return None

    def _stop_for_acoustic_failure(self, message):
        """Stop microphone ASR after an unrecoverable detector error."""
        self._fatal_error = message
        self.get_logger().error(message)
        self._running = False
        try:
            rclpy.shutdown()
        except Exception:
            pass

    def _begin_wake_verification(self, score):
        """Atomically open a new acoustic wake session."""
        with self._wake_state_lock:
            if self._phase != 'wait_wake':
                return None
            self._wake_generation += 1
            generation = self._wake_generation
            self._phase = 'verify_wake'
            self._command_deadline = 0.0
            self._pending_in_time_commands = 0
        discarded = 0
        while True:
            try:
                self._audio_queue.get_nowait()
                discarded += 1
            except queue.Empty:
                break
        if discarded:
            self.get_logger().debug(
                f'Discarded {discarded} stale queued segment(s) before '
                f'wake session {generation}.')
        self.get_logger().info(
            f'Acoustic wake candidate detected (score={score:.3f}, '
            f'session={generation}); awaiting exact Whisper verification.')
        return generation

    def _note_active_segment_started(self, started_at):
        """Remember a possible command that began while wake is verified."""
        if not getattr(self, 'audio_wake_enabled', False):
            return
        with self._wake_state_lock:
            if self._phase not in ('verify_wake', 'command'):
                return
            self._active_segment_generation = self._wake_generation
            self._active_segment_started_at = started_at

    def _note_active_segment_finished(self, started_at):
        if not getattr(self, 'audio_wake_enabled', False):
            return
        with self._wake_state_lock:
            if self._active_segment_started_at == started_at:
                self._active_segment_generation = None
                self._active_segment_started_at = None

    def _enqueue_segment(self, speech_buffer, wake_generation=None,
                         segment_started_at=None):
        """
        Route a segment into the bounded transcription queue.

        In acoustic mode only the wake candidate and segments from its active
        session are admitted. Idle background speech is discarded here.
        """
        cut_t = time.monotonic()   # VAD closed this segment (utterance end)
        if not speech_buffer:
            return
        audio_data = np.concatenate(speech_buffer, axis=0).flatten()
        duration = len(audio_data) / self.capture_sample_rate
        if duration < self.min_speech_seconds:
            self.get_logger().debug(
                f'Speech too short ({duration:.2f}s), skipping')
            if wake_generation is not None:
                self._close_acoustic_session(
                    wake_generation, 'wake candidate was too short')
            return

        if not getattr(self, 'audio_wake_enabled', False):
            self.get_logger().info(
                f'Captured {duration:.1f}s of speech, queued.')
            self._put_legacy_audio(audio_data)
            return

        started_at = (
            segment_started_at
            if segment_started_at is not None else time.monotonic())
        deadline_reserved = False
        if wake_generation is not None:
            kind = 'wake'
            generation = wake_generation
        else:
            with self._wake_state_lock:
                phase = self._phase
                if phase not in ('verify_wake', 'command'):
                    self.get_logger().debug(
                        f'Discarded {duration:.1f}s background segment '
                        '(no acoustic wake hit).')
                    return
                generation = self._wake_generation
                deadline_reserved = (
                    phase == 'verify_wake' or
                    started_at <= self._command_deadline)
                if deadline_reserved:
                    self._pending_in_time_commands += 1
            kind = 'command'

        item = AudioQueueItem(
            kind=kind,
            generation=generation,
            audio=audio_data,
            started_at=started_at,
            deadline_reserved=deadline_reserved,
            speech_end_at=cut_t,
        )
        try:
            queued = self._put_acoustic_item(item)
        except Exception:
            self._release_deadline_reservation(item)
            raise
        if not queued:
            self._release_deadline_reservation(item)
            return
        self.get_logger().info(
            f'Captured {duration:.1f}s {kind} segment for '
            f'wake session {generation}, queued.')

    def _put_legacy_audio(self, audio_data):
        """Preserve the bounded transcription-first diagnostic behaviour."""
        if self._audio_queue.qsize() >= self._max_queued_segments:
            try:
                self._audio_queue.get_nowait()
            except queue.Empty:
                pass
        self._audio_queue.put(audio_data)

    def _put_acoustic_item(self, item):
        """Bound a deliberate wake session without ever dropping its wake item."""
        if self._audio_queue.qsize() >= self._max_queued_segments:
            if item.kind == 'command':
                self.get_logger().warn(
                    f'Wake session {item.generation} command queue full; '
                    'dropping newest segment.')
                return False
            # A new wake starts from WAIT_WAKE, so anything already queued is
            # stale. Clear it before inserting the verification candidate.
            while True:
                try:
                    self._audio_queue.get_nowait()
                except queue.Empty:
                    break
        self._audio_queue.put(item)
        return True

    def _release_deadline_reservation(self, item):
        """Release one generation-bound in-time command reservation."""
        if (item.kind != 'command' or
                not item.deadline_reserved):
            return
        with self._wake_state_lock:
            if item.generation != self._wake_generation:
                return
            if self._pending_in_time_commands > 0:
                self._pending_in_time_commands -= 1

    # ── Transcription worker + two-stage wake FSM ───────────────────

    def _transcribe_worker(self):
        """
        Transcribe queued segments and drive the wake FSM.

        Run on a separate thread so capture continues while Whisper decodes.
        """
        if getattr(self, 'audio_wake_enabled', False):
            self._transcribe_worker_acoustic()
            return
        self._transcribe_worker_legacy()

    def _transcribe_worker_legacy(self):
        """Original transcription-first flow, kept only for explicit diagnostics."""
        while self._running and rclpy.ok():
            if self._phase == 'command':
                remaining = self._command_deadline - time.monotonic()
                if remaining <= 0:
                    self._handle_segment(None)      # window elapsed -> disarm
                    continue
                try:
                    audio = self._audio_queue.get(timeout=remaining)
                except queue.Empty:
                    self._handle_segment(None)      # window elapsed -> disarm
                    continue
                text = self._transcribe(audio)
                if not text:
                    continue    # empty/garbage segment: keep the window open
                self._handle_segment(text)          # -> publish command, disarm
            else:  # wait_wake — block for a segment, waking periodically to exit
                try:
                    audio = self._audio_queue.get(timeout=0.5)
                except queue.Empty:
                    continue
                text = self._transcribe(audio)
                self._handle_segment(text)          # wake decision

    def _transcribe_worker_acoustic(self):
        """Consume only candidates admitted by the acoustic wake gate."""
        while self._running and rclpy.ok():
            try:
                item = self._audio_queue.get(timeout=0.2)
            except queue.Empty:
                # Queued items are checked first because the deadline applies to
                # when speech *started*, not when the finished segment happens
                # to be dequeued. An in-time item carries its own start time.
                self._expire_acoustic_command_window()
                continue
            if not isinstance(item, AudioQueueItem):
                self.get_logger().warn(
                    'Discarded untagged audio from acoustic wake queue.')
                continue
            self._process_acoustic_queue_item(item)

    def _process_acoustic_queue_item(self, item):
        """Transcribe one tagged item and always release its deadline claim."""
        try:
            with self._wake_state_lock:
                current_generation = self._wake_generation
                phase = self._phase
                deadline = self._command_deadline
            if item.generation != current_generation:
                self.get_logger().debug(
                    f'Discarded stale {item.kind} segment from wake session '
                    f'{item.generation}.')
                return

            if item.kind == 'wake':
                if phase != 'verify_wake':
                    self.get_logger().debug(
                        f'Discarded unexpected wake segment in phase {phase}.')
                    return
                self._handle_acoustic_wake_transcript(
                    item, self._transcribe(item.audio))
                return

            if item.kind != 'command':
                self.get_logger().error(
                    f'Discarded unknown audio queue item kind: {item.kind}')
                return
            if phase != 'command':
                # FIFO ordering guarantees that the wake item is processed
                # before its followers. A different phase therefore means this
                # command belongs to a rejected/closed session.
                self.get_logger().debug(
                    f'Discarded command segment while phase is {phase}.')
                return
            if item.started_at > deadline:
                self._close_acoustic_session(
                    item.generation,
                    'command started after the six-second window',
                )
                return

            text = self._transcribe(item.audio)
            if not text:
                # A cough/noise segment does not consume the command window.
                return
            action = plan_wake_segment(
                'command', text, self.wake_words, self.wake_word_fuzzy,
                self.two_stage_wake)
            command = (
                action[1] if action[0] == 'command'
                else correct_addressed_command(text))
            if self._close_acoustic_session(
                    item.generation, 'command accepted'):
                self._publish_command(command)
                self._emit_timing(item, command)
        finally:
            self._release_deadline_reservation(item)

    def _handle_acoustic_wake_transcript(self, item, text):
        """Verify the acoustic hit with Whisper and arm/publish/reject it."""
        generation = item.generation
        with self._wake_state_lock:
            if (generation != self._wake_generation or
                    self._phase != 'verify_wake'):
                return

        if self.wake_require_separate_command:
            if not is_exact_wake_utterance(text, self.wake_words):
                rendered = text if text else '<empty>'
                self.get_logger().info(
                    'Acoustic wake candidate rejected by exact verification: '
                    f'"{rendered}" (expected the wake word alone).')
                self._close_acoustic_session(
                    generation, 'exact wake verification rejected')
                return
            self._arm_acoustic_session(generation)
            return

        # Compatibility mode: retain the previous one-breath behaviour when
        # strict separation is explicitly disabled.
        action = plan_wake_segment(
            'wait_wake', text, self.wake_words, self.wake_word_fuzzy,
            self.two_stage_wake)
        if action[0] == 'arm':
            self._arm_acoustic_session(generation)
        elif action[0] == 'command':
            if self._close_acoustic_session(
                    generation, 'inline command accepted'):
                self._publish_command(action[1])
                self._emit_timing(item, action[1])
        else:
            rendered = text if text else '<empty>'
            self.get_logger().info(
                f'Acoustic wake candidate rejected after transcription: '
                f'"{rendered}".')
            self._close_acoustic_session(
                generation, 'wake verification rejected')

    def _arm_acoustic_session(self, generation):
        with self._wake_state_lock:
            if (generation != self._wake_generation or
                    self._phase != 'verify_wake'):
                return
            self._phase = 'command'
            self._command_deadline = (
                time.monotonic() + self.command_window_sec)
            # Serialize status with session state. Otherwise a disconnect could
            # publish idle between unlock and this listening update, leaving
            # the HRI stuck on a stale "speak now" indication.
            self._publish_status('listening')
        self.get_logger().info(
            'Wake word verified — speak your command within '
            f'{self.command_window_sec:g} seconds.')

    def _expire_acoustic_command_window(self):
        """Disarm at the deadline unless a command began before it."""
        with self._wake_state_lock:
            generation = self._wake_generation
        self._close_acoustic_session(
            generation,
            'command window expired',
            require_expired_idle=True,
        )

    def _close_acoustic_session(
            self, generation, reason, require_expired_idle=False):
        """Close one session and invalidate every queued follower atomically."""
        if not getattr(self, 'audio_wake_enabled', False):
            return False
        with self._wake_state_lock:
            if generation != self._wake_generation:
                return False
            if self._phase == 'resetting':
                return False
            if require_expired_idle:
                now = time.monotonic()
                in_time_segment_active = (
                    self._active_segment_generation == generation and
                    self._active_segment_started_at is not None and
                    self._active_segment_started_at <= self._command_deadline)
                if (self._phase != 'command' or
                        now < self._command_deadline or
                        in_time_segment_active or
                        self._pending_in_time_commands > 0):
                    return False
            old_phase = self._phase
            self._phase = 'resetting'
            self._command_deadline = 0.0
            self._wake_generation += 1
            reset_generation = self._wake_generation
            self._active_segment_generation = None
            self._active_segment_started_at = None
            self._pending_in_time_commands = 0
            if old_phase == 'command':
                self._publish_status('')
        self.get_logger().info(
            f'Wake session {generation} closed: {reason}.')
        detector = getattr(self, '_wake_detector', None)
        reset_ok = True
        if detector is not None:
            try:
                with self._detector_lock:
                    detector.reset()
            except Exception as exc:
                reset_ok = False
                self._stop_for_acoustic_failure(
                    'Failed to reset acoustic wake detector; ASR is '
                    f'stopping: {exc}')
        if reset_ok:
            with self._wake_state_lock:
                if (self._phase == 'resetting' and
                        self._wake_generation == reset_generation):
                    self._phase = 'wait_wake'
        return reset_ok

    def _reset_acoustic_wake_runtime(self, reason):
        """Reset detector/session after a microphone disconnect or hot-swap."""
        if not getattr(self, 'audio_wake_enabled', False):
            return
        with self._wake_state_lock:
            generation = self._wake_generation
            active = self._phase != 'wait_wake'
        if active:
            self._close_acoustic_session(generation, reason)
            return
        detector = getattr(self, '_wake_detector', None)
        if detector is not None:
            try:
                with self._detector_lock:
                    detector.reset()
            except Exception as exc:
                self._stop_for_acoustic_failure(
                    'Failed to reset acoustic wake detector; ASR is '
                    f'stopping: {exc}')

    def _publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_publisher.publish(msg)

    def _publish_command(self, text):
        msg = String()
        msg.data = text
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: "{text}"')

    def _emit_timing(self, item, text):
        """Diagnostic only: publish per-command latency timestamps on /asr_timing.

        All times are asr_node's monotonic clock, so an external logger can take
        the pure transcription latency as t_publish - t_speech_end directly from
        the message (host/DDS-latency independent). No effect on the command path.
        """
        try:
            audio_s = float(len(item.audio)) / float(self.capture_sample_rate)
        except Exception:
            audio_s = 0.0
        payload = {
            't_speech_start': round(float(item.started_at), 6),
            't_speech_end': round(float(item.speech_end_at), 6),
            't_publish': round(time.monotonic(), 6),
            'audio_s': round(audio_s, 4),
            'text': text,
        }
        m = String()
        m.data = json.dumps(payload)
        self._timing_pub.publish(m)

    def _handle_segment(self, text):
        """Apply the two-stage wake decision (pure planner) + its side effects."""
        was_command_phase = (self._phase == 'command')
        action = plan_wake_segment(
            self._phase, text, self.wake_words, self.wake_word_fuzzy,
            self.two_stage_wake)

        # Leaving the command phase always disarms the listening feedback first.
        if was_command_phase:
            self._phase = 'wait_wake'
            self._publish_status('')

        kind = action[0]
        if kind == 'command':
            self._publish_command(action[1])
        elif kind == 'arm':
            self.get_logger().info('Wake word detected — speak your command.')
            self._phase = 'command'
            self._command_deadline = (
                time.monotonic() + self.command_window_sec)
            self._publish_status('listening')
        elif kind == 'disarm':
            self.get_logger().info('Command window expired — disarmed.')
        elif kind == 'ignore' and text and not was_command_phase:
            self.get_logger().info(f'Ignored (no command): "{text}"')

    def _transcribe(self, audio_data):
        """Transcribe audio buffer using faster-whisper."""
        if self._model is None:
            return None

        try:
            audio_input = audio_data
            if self.capture_sample_rate != self.sample_rate:
                # A numpy array is assumed by faster-whisper to already be at
                # Whisper's native rate. Passing a WAV file-like object instead
                # invokes its decoder/resampler, preserving the real duration.
                pcm = (
                    np.clip(audio_data, -1.0, 1.0) * np.iinfo(np.int16).max
                ).astype('<i2')
                audio_input = io.BytesIO()
                with wave.open(audio_input, 'wb') as wav_file:
                    wav_file.setnchannels(1)
                    wav_file.setsampwidth(2)
                    wav_file.setframerate(self.capture_sample_rate)
                    wav_file.writeframes(pcm.tobytes())
                audio_input.seek(0)

            # Greedy decoding (beam_size=1) is ~2x faster than beam search.
            # The rest is about bailing FAST on non-speech and NEVER getting stuck
            # in Whisper's repetition spiral ("awl, awl, awl, ..." for 30+ s, which
            # blocks the whole transcription queue). Silero vad_filter drops
            # non-speech before decoding; condition_on_previous_text=False avoids
            # cross-segment priming; no_repeat_ngram_size stops the model from
            # looping a short phrase; temperature=0.0 disables the multi-temperature
            # fallback whose retries are what make a bad segment take tens of
            # seconds; max_new_tokens bounds the worst case (a command is short).
            segments, info = self._model.transcribe(
                audio_input,
                language=self.language,
                beam_size=1,
                temperature=0.0,
                no_repeat_ngram_size=3,
                max_new_tokens=64,
                vad_filter=True,
                vad_parameters=dict(min_silence_duration_ms=300),
                condition_on_previous_text=False,
                no_speech_threshold=self.no_speech_threshold,
                compression_ratio_threshold=2.4,
                log_prob_threshold=-1.0,
                # Vocabulary bias toward our command set — see the parameter.
                initial_prompt=self.initial_prompt,
            )

            full_text = ' '.join(seg.text.strip() for seg in segments).strip()

            if full_text:
                self.get_logger().info(
                    f'Transcription ({info.language}, p={info.language_probability:.2f}): '
                    f'"{full_text}"'
                )
            return full_text if full_text else None

        except Exception as e:
            self.get_logger().error(f'Transcription failed: {e}')
            return None

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ASRNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if node._fatal_error:
        raise RuntimeError(node._fatal_error)


if __name__ == '__main__':
    main()
