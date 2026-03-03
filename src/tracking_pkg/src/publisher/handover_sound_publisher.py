#!/usr/bin/env python3

import os
import queue
import shutil
import subprocess
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String


class HandoverSoundPublisher(Node):
    def __init__(self):
        super().__init__('handover_sound_publisher')
        self.subscription = self.create_subscription(
            String,
            '/handover_event',
            self._event_callback,
            10,
        )

        self.player_name, self.player_binary = self._select_audio_backend()
        if self.player_binary is None:
            self.get_logger().error("No supported audio backend found (paplay, pw-play, aplay). Audio feedback is disabled.")
        else:
            self._validate_audio_backend()
            self.get_logger().info(f"Using audio backend: {self.player_name} ({self.player_binary})")

        try:
            package_share = get_package_share_directory('tracking_pkg')
        except Exception as exc:
            package_share = ''
            self.get_logger().error(f'Failed to resolve package share directory: {exc}')

        sounds_dir = os.path.join(package_share, 'sounds')
        self.event_to_sound = {
            'gesture_detected': os.path.join(sounds_dir, 'gesture_detected.wav'),
            'reachability:unreachable_plan_failed': os.path.join(sounds_dir, 'unreachable.wav'),
            'reachability:unreachable_execution_failed': os.path.join(sounds_dir, 'unreachable.wav'),
        }

        self.event_debounce_sec = {
            'gesture_detected': 0.20,
            'reachability:unreachable_plan_failed': 0.40,
            'reachability:unreachable_execution_failed': 0.40,
        }
        self.last_event_time = {}
        self.valid_sound_paths = self._validate_sound_files()
        self.playback_queue = queue.Queue(maxsize=8)
        self.inter_sound_gap_sec = 0.00
        self.batch_window_sec = 0.35
        self.backend_reopen_gap_sec = 0.75
        self.max_batch_size = 4
        self.last_playback_end_time = 0.0

        self.playback_thread = threading.Thread(target=self._playback_worker, daemon=True)
        self.playback_thread.start()

        self.get_logger().info('Handover sound publisher initialized.')

    def _event_callback(self, msg: String) -> None:
        event_name = msg.data
        if event_name not in self.event_to_sound:
            return

        now = time.monotonic()
        last_time = self.last_event_time.get(event_name, 0.0)
        debounce_sec = self.event_debounce_sec.get(event_name, 0.20)
        if now - last_time < debounce_sec:
            return

        sound_path = self.event_to_sound[event_name]
        if sound_path not in self.valid_sound_paths:
            return

        if self.player_binary is None:
            return

        if self.playback_queue.full():
            self.get_logger().warn(f'Playback queue is full; dropping newest sound event: {event_name}')
            return

        try:
            self.playback_queue.put_nowait((event_name, sound_path))
            self.last_event_time[event_name] = now
        except queue.Full:
            self.get_logger().warn(f'Playback queue is full; dropping newest sound event: {event_name}')

    def _playback_worker(self) -> None:
        while rclpy.ok():
            try:
                event_name, sound_path = self.playback_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            try:
                batch = [(event_name, sound_path)]
                batch_deadline = time.monotonic() + self.batch_window_sec

                while len(batch) < self.max_batch_size:
                    remaining = batch_deadline - time.monotonic()
                    if remaining <= 0.0:
                        break
                    try:
                        next_item = self.playback_queue.get(timeout=remaining)
                        batch.append(next_item)
                    except queue.Empty:
                        break

                if self.player_name == 'aplay':
                    self._play_batch_with_aplay(batch)
                else:
                    for queued_event_name, queued_sound_path in batch:
                        self._play_single_sound(queued_event_name, queued_sound_path)
                        time.sleep(self.inter_sound_gap_sec)
            except Exception as exc:
                self.get_logger().error(f'Failed to play sound batch starting with {sound_path}: {exc}')

    def _play_batch_with_aplay(self, batch) -> None:
        self._enforce_backend_reopen_gap()
        command = [self.player_binary, '-q']
        command.extend(sound_path for _, sound_path in batch)
        completed = subprocess.run(
            command,
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        self.last_playback_end_time = time.monotonic()

        event_names = ', '.join(event_name for event_name, _ in batch)
        if completed.returncode == 0:
            self.get_logger().info(f'Played sound batch for events: {event_names}')
        else:
            self.get_logger().warn(
                f'Audio backend {self.player_name} returned non-zero exit code {completed.returncode} for events: {event_names}'
            )

    def _play_single_sound(self, event_name: str, sound_path: str) -> None:
        self._enforce_backend_reopen_gap()
        completed = subprocess.run(
            self._build_playback_command(sound_path),
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        self.last_playback_end_time = time.monotonic()

        if completed.returncode == 0:
            self.get_logger().info(f'Played sound for event: {event_name}')
        else:
            self.get_logger().warn(
                f'Audio backend {self.player_name} returned non-zero exit code {completed.returncode} for event: {event_name}'
            )

    def _enforce_backend_reopen_gap(self) -> None:
        if self.last_playback_end_time <= 0.0:
            return
        elapsed = time.monotonic() - self.last_playback_end_time
        remaining = self.backend_reopen_gap_sec - elapsed
        if remaining > 0.0:
            time.sleep(remaining)

    def _select_audio_backend(self):
        for player_name in ('aplay', 'pw-play', 'paplay'):
            binary = shutil.which(player_name)
            if binary is not None:
                return player_name, binary
        return None, None

    def _validate_audio_backend(self) -> None:
        try:
            subprocess.run(
                [self.player_binary, '--version'],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception as exc:
            self.get_logger().warn(f'Audio backend validation failed for {self.player_name}: {exc}')

    def _validate_sound_files(self):
        valid_paths = set()
        logged_paths = set()
        for sound_path in self.event_to_sound.values():
            if sound_path in logged_paths:
                continue
            logged_paths.add(sound_path)
            if not os.path.isfile(sound_path):
                self.get_logger().warn(f'Sound file missing: {sound_path}')
                continue
            file_size = os.path.getsize(sound_path)
            self.get_logger().info(f'Sound file ready: {sound_path} ({file_size} bytes)')
            valid_paths.add(sound_path)
        return valid_paths

    def _build_playback_command(self, sound_path: str):
        if self.player_name == 'paplay':
            return [self.player_binary, sound_path]
        if self.player_name == 'pw-play':
            return [self.player_binary, sound_path]
        return [self.player_binary, '-q', sound_path]


def main(args=None):
    rclpy.init(args=args)
    node = HandoverSoundPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
