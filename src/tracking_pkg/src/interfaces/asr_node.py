#!/usr/bin/env python3
"""
ASR (Automatic Speech Recognition) Node
========================================
Captures audio from microphone via sounddevice, detects speech segments
using energy-based VAD, and transcribes them with faster-whisper (local, free).

Pipeline:
  Microphone (sounddevice) → energy-based VAD → WAV buffer
    → faster-whisper transcription → /user_speech

Publishers:
  /user_speech (std_msgs/String) - transcribed speech segments

Parameters:
  whisper_model (str): Model size: tiny.en, base.en, small.en, medium, large-v3
                       Recommendation: 'base.en' for CPU (fast, English-only)
  language (str): Expected language code (en) or empty for auto-detect
  sample_rate (int): Whisper input sample rate in Hz (default: 16000)
  capture_sample_rate (int): Native microphone rate; 0 uses sample_rate
  silence_threshold_seconds (float): Seconds of silence to end a segment
  energy_threshold (float): RMS energy threshold for speech detection
  min_speech_seconds (float): Minimum speech duration to trigger transcription
  device_index (int): Explicit microphone index (-1 = automatic selection)
  audio_device (str): Explicit name substring; overrides automatic selection
  audio_device_candidates (str[]): Auto-detected input name substrings
  audio_device_candidate_rates (int[]): Native rates paired with candidates
  audio_device_retry_seconds (float): Rescan interval while no unique input exists

Install:
  pip install faster-whisper sounddevice numpy
"""

import io
import time
import wave
import queue
import numpy as np
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from wake_word import plan_wake_segment


class AudioDeviceSelectionError(RuntimeError):
    """Raised when automatic input selection is ambiguous or misconfigured."""


def select_supported_audio_device(devices, candidates, sample_rates):
    """Return the single supported input as ``(index, name, rate, match)``.

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
        # Two-stage wake word: say "robot" alone -> the display shows "speak
        # now" -> then say the command. Solves the wake word getting lost inside
        # a fast "robot needle holder". A one-breath "robot needle holder" still
        # works (published directly). False -> old single-segment behaviour.
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
        self.initial_prompt = self.get_parameter('initial_prompt').value or None
        self.two_stage_wake = bool(self.get_parameter('two_stage_wake').value)
        self.command_window_sec = float(
            self.get_parameter('command_window_sec').value)
        self.no_speech_threshold = float(
            self.get_parameter('no_speech_threshold').value)

        # Publishers
        self.publisher = self.create_publisher(String, 'user_speech', 10)
        # HRI status for the visual display: 'listening' (armed, waiting for the
        # command -> amber "speak now") or '' (idle). A pure feedback channel.
        # Latched: the HRI display (which boots faster than the Whisper model
        # loads) still gets the last status — including the initial 'ready' —
        # whenever it (re)subscribes.
        self.status_publisher = self.create_publisher(
            String, 'asr_status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        # Two-stage wake FSM state (only used when two_stage_wake and wake_words).
        self._phase = 'wait_wake'      # 'wait_wake' | 'command'
        self._command_deadline = 0.0

        # Audio state
        self._audio_queue = queue.Queue()
        self._model = None
        self._running = True

        capture_rate_description = (
            'auto' if self._auto_select_audio
            else f'{self.capture_sample_rate}Hz')
        self.get_logger().info(
            f'ASRNode starting (model={self.whisper_model_size}, '
            f'device={self.asr_device}/{self.compute_type}, '
            f'lang={self.language or "auto"}, '
            f'capture_rate={capture_rate_description}, '
            f'whisper_rate={self.sample_rate}Hz, '
            f'silence={self.silence_threshold}s, energy={self.energy_threshold})'
        )

        # Start audio + transcription threads
        self._listen_thread = Thread(target=self._listen_loop, daemon=True)
        self._listen_thread.start()

    def _load_model(self):
        """Load faster-whisper model (runs on first audio segment).

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
        Main audio capture + VAD + transcription loop.
        Runs in a dedicated thread to not block ROS callbacks.
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

        chunk_duration = 0.1  # 100ms chunks

        _no_mic_warned = False
        while self._running and rclpy.ok():
            try:
                # In the command phase, cap how long we wait for the command to
                # begin so a wake word with no follow-up disarms on its own.
                max_wait = None
                if self._phase == 'command':
                    max_wait = max(0.0, self._command_deadline - time.time())
                text = self._capture_and_transcribe(sd, chunk_duration, max_wait)
                self._handle_segment(text)
                _no_mic_warned = False
            except sd.PortAudioError as e:
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
                self.get_logger().error(f'Listen loop error: {e}')
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
        self.device_index = index
        self.capture_sample_rate = sample_rate

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

    def _capture_and_transcribe(self, sd, chunk_duration,
                                max_wait_for_speech=None):
        """
        Capture one speech segment and transcribe it. Returns the transcript
        (str) or None (nothing usable / timed out before speech began).

        Uses a simple energy-based VAD:
        1. Wait for energy above threshold (speech start)
        2. Accumulate audio while energy stays above threshold
        3. When silence exceeds silence_threshold → segment complete
        4. Transcribe if long enough

        max_wait_for_speech: if set, return None when no speech has STARTED
        within this many seconds (used to disarm the command window). Once
        speech begins it is always captured to completion.
        """
        speech_buffer = []
        is_speaking = False
        silence_start = None

        self.get_logger().info('Listening for speech...')

        # Recomputed for every stream so a Samson/Jieli hot-swap also updates
        # the 100 ms block size from 1600 to 4800 samples (or vice versa).
        chunk_samples = audio_capture_block_size(
            self.capture_sample_rate, chunk_duration)

        wait_start = time.time()
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

                # Compute RMS energy
                rms = np.sqrt(np.mean(audio_chunk ** 2))

                if rms >= self.energy_threshold:
                    # Speech detected
                    if not is_speaking:
                        is_speaking = True
                        self.get_logger().debug('Speech started')
                    speech_buffer.append(audio_chunk.copy())
                    silence_start = None
                elif is_speaking:
                    # Below threshold but was speaking → count silence
                    speech_buffer.append(audio_chunk.copy())
                    if silence_start is None:
                        silence_start = time.time()
                    elif time.time() - silence_start >= self.silence_threshold:
                        # Silence long enough → end of speech segment
                        break
                elif (max_wait_for_speech is not None
                      and (time.time() - wait_start) >= max_wait_for_speech):
                    # Armed for a command but nobody spoke in time.
                    return None

        if not speech_buffer:
            return None

        # Concatenate speech buffer
        audio_data = np.concatenate(speech_buffer, axis=0).flatten()
        duration = len(audio_data) / self.capture_sample_rate

        if duration < self.min_speech_seconds:
            self.get_logger().debug(
                f'Speech too short ({duration:.2f}s < {self.min_speech_seconds}s), skipping'
            )
            return None

        self.get_logger().info(f'Captured {duration:.1f}s of speech, transcribing...')
        return self._transcribe(audio_data)

    # ── Two-stage wake FSM ──────────────────────────────────────────

    def _publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_publisher.publish(msg)

    def _publish_command(self, text):
        msg = String()
        msg.data = text
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: "{text}"')

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
            self._command_deadline = time.time() + self.command_window_sec
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
            # The rest is about bailing FAST on non-speech: background noise used
            # to trigger a segment and then take up to 30 s to decode (Whisper
            # looping/hallucinating), blocking the wake word. Silero vad_filter
            # drops non-speech before decoding, condition_on_previous_text=False
            # stops the repetition spiral, and the thresholds abort low-confidence
            # / repetitive output early.
            segments, info = self._model.transcribe(
                audio_input,
                language=self.language,
                beam_size=1,
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
