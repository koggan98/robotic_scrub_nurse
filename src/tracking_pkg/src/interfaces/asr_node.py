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

        self.whisper_model_size = self.get_parameter('whisper_model').value
        self.language = self.get_parameter('language').value or None
        self.sample_rate = int(self.get_parameter('sample_rate').value)
        self.silence_threshold = float(self.get_parameter('silence_threshold_seconds').value)
        self.energy_threshold = float(self.get_parameter('energy_threshold').value)
        self.min_speech_seconds = float(self.get_parameter('min_speech_seconds').value)
        device_idx = int(self.get_parameter('device_index').value)
        self.device_index = None if device_idx < 0 else device_idx

        # Publisher
        self.publisher = self.create_publisher(String, 'user_speech', 10)

        # Audio state
        self._audio_queue = queue.Queue()
        self._model = None
        self._running = True

        self.get_logger().info(
            f'ASRNode starting (model={self.whisper_model_size}, '
            f'lang={self.language or "auto"}, rate={self.sample_rate}Hz, '
            f'silence={self.silence_threshold}s, energy={self.energy_threshold})'
        )

        # Start audio + transcription threads
        self._listen_thread = Thread(target=self._listen_loop, daemon=True)
        self._listen_thread.start()

    def _load_model(self):
        """Load faster-whisper model (runs on first audio segment)."""
        try:
            from faster_whisper import WhisperModel
            self.get_logger().info(
                f'Loading faster-whisper model "{self.whisper_model_size}" on CPU...'
            )
            self._model = WhisperModel(
                self.whisper_model_size,
                device='cpu',
                compute_type='int8',
            )
            self.get_logger().info('Whisper model loaded successfully')
            return True
        except ImportError:
            self.get_logger().error(
                'faster-whisper not installed. Run: pip install faster-whisper'
            )
            return False
        except Exception as e:
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

        while self._running and rclpy.ok():
            try:
                self._capture_and_transcribe(sd, chunk_samples, chunk_duration)
            except sd.PortAudioError as e:
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
        if text:
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
