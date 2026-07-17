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
  sample_rate (int): Audio sample rate in Hz (default: 16000, Whisper native)
  silence_threshold_seconds (float): Seconds of silence to end a segment
  energy_threshold (float): RMS energy threshold for speech detection
  min_speech_seconds (float): Minimum speech duration to trigger transcription
  device_index (int): Microphone device index (-1 = system default)

Install:
  pip install faster-whisper sounddevice numpy
"""

import io
import time
import wave
import queue
import tempfile
import numpy as np
from threading import Thread

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from wake_word import strip_wake_word


class ASRNode(Node):
    def __init__(self):
        super().__init__('asr_node')

        # Parameters
        self.declare_parameter('whisper_model', 'base')
        self.declare_parameter('language', 'de')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('silence_threshold_seconds', 0.8)
        self.declare_parameter('energy_threshold', 0.015)
        self.declare_parameter('min_speech_seconds', 0.5)
        self.declare_parameter('device_index', -1)
        # Select the input device by NAME substring (passed straight to sounddevice),
        # e.g. 'Samson'. Preferred over device_index: robust to index changes and works
        # when PulseAudio exposes no capture source (direct ALSA). '' -> use device_index.
        self.declare_parameter('audio_device', '')
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

        self.whisper_model_size = self.get_parameter('whisper_model').value
        self.language = self.get_parameter('language').value or None
        self.sample_rate = int(self.get_parameter('sample_rate').value)
        self.silence_threshold = float(self.get_parameter('silence_threshold_seconds').value)
        self.energy_threshold = float(self.get_parameter('energy_threshold').value)
        self.min_speech_seconds = float(self.get_parameter('min_speech_seconds').value)
        device_idx = int(self.get_parameter('device_index').value)
        audio_device = self.get_parameter('audio_device').value
        # A name substring wins over the numeric index; sounddevice accepts str/int/None.
        self.device_index = audio_device if audio_device else (
            None if device_idx < 0 else device_idx)
        self.asr_device = self.get_parameter('device').value or 'cuda'
        compute_type = self.get_parameter('compute_type').value
        self.cpu_threads = int(self.get_parameter('cpu_threads').value)
        self.compute_type = compute_type or (
            'float16' if str(self.asr_device).startswith('cuda') else 'int8')
        self.wake_words = list(self.get_parameter('wake_words').value or [])
        self.wake_word_fuzzy = float(self.get_parameter('wake_word_fuzzy').value)

        # Publisher
        self.publisher = self.create_publisher(String, 'user_speech', 10)

        # Audio state
        self._audio_queue = queue.Queue()
        self._model = None
        self._running = True

        self.get_logger().info(
            f'ASRNode starting (model={self.whisper_model_size}, '
            f'device={self.asr_device}/{self.compute_type}, '
            f'lang={self.language or "auto"}, rate={self.sample_rate}Hz, '
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

        # Load model before starting capture
        if not self._load_model():
            return

        # Log available devices for debugging
        self.get_logger().info(f'Audio devices:\n{sd.query_devices()}')
        if self.device_index is not None:
            self.get_logger().info(f'Using device index: {self.device_index}')
        else:
            self.get_logger().info(
                f'Using default input device: {sd.query_devices(kind="input")["name"]}'
            )

        chunk_duration = 0.1  # 100ms chunks
        chunk_samples = int(self.sample_rate * chunk_duration)

        _no_mic_warned = False
        while self._running and rclpy.ok():
            try:
                self._capture_and_transcribe(sd, chunk_samples, chunk_duration)
                _no_mic_warned = False
            except sd.PortAudioError as e:
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

    def _capture_and_transcribe(self, sd, chunk_samples, chunk_duration):
        """
        Capture one speech segment and transcribe it.

        Uses a simple energy-based VAD:
        1. Wait for energy above threshold (speech start)
        2. Accumulate audio while energy stays above threshold
        3. When silence exceeds silence_threshold → segment complete
        4. Transcribe if long enough
        """
        speech_buffer = []
        is_speaking = False
        silence_start = None

        self.get_logger().info('Listening for speech...')

        with sd.InputStream(
            samplerate=self.sample_rate,
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

        if not speech_buffer:
            return

        # Concatenate speech buffer
        audio_data = np.concatenate(speech_buffer, axis=0).flatten()
        duration = len(audio_data) / self.sample_rate

        if duration < self.min_speech_seconds:
            self.get_logger().debug(
                f'Speech too short ({duration:.2f}s < {self.min_speech_seconds}s), skipping'
            )
            return

        self.get_logger().info(f'Captured {duration:.1f}s of speech, transcribing...')

        # Transcribe
        text = self._transcribe(audio_data)
        if not text:
            return

        # Wake-word gate: only utterances addressed to the robot go through.
        if self.wake_words:
            command = strip_wake_word(text, self.wake_words,
                                      self.wake_word_fuzzy)
            if command is None:
                self.get_logger().info(f'No wake word — ignored: "{text}"')
                return
            if not command:
                self.get_logger().info('Wake word only — no command, ignored.')
                return
            text = command

        msg = String()
        msg.data = text
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: "{text}"')

    def _transcribe(self, audio_data):
        """Transcribe audio buffer using faster-whisper."""
        if self._model is None:
            return None

        try:
            # Speed-tuned: energy-based VAD already trims silence upstream,
            # so faster-whisper's internal Silero VAD pass is redundant.
            # Greedy decoding (beam_size=1) is ~2x faster than beam search.
            segments, info = self._model.transcribe(
                audio_data,
                language=self.language,
                beam_size=1,
                vad_filter=False,
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
