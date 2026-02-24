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

        self.aplay_binary = shutil.which('aplay')
        if self.aplay_binary is None:
            self.get_logger().error("'aplay' not found. Audio feedback is disabled.")

        try:
            package_share = get_package_share_directory('tracking_pkg')
        except Exception as exc:
            package_share = ''
            self.get_logger().error(f'Failed to resolve package share directory: {exc}')

        sounds_dir = os.path.join(package_share, 'sounds')
        self.event_to_sound = {
            'gesture_detected': os.path.join(sounds_dir, 'gesture_detected.wav'),
            'reachability:unreachable_plan_failed': os.path.join(sounds_dir, 'unreachable.wav'),
        }

        self.event_debounce_sec = 0.5
        self.last_event_time = {}
        self.playback_queue = queue.Queue(maxsize=1)

        self.playback_thread = threading.Thread(target=self._playback_worker, daemon=True)
        self.playback_thread.start()

        self.get_logger().info('Handover sound publisher initialized.')

    def _event_callback(self, msg: String) -> None:
        event_name = msg.data
        if event_name not in self.event_to_sound:
            return

        now = time.monotonic()
        last_time = self.last_event_time.get(event_name, 0.0)
        if now - last_time < self.event_debounce_sec:
            return

        self.last_event_time[event_name] = now
        sound_path = self.event_to_sound[event_name]

        if not os.path.isfile(sound_path):
            self.get_logger().warn(f'Sound file missing for event {event_name}: {sound_path}')
            return

        if self.aplay_binary is None:
            return

        if self.playback_queue.full():
            try:
                self.playback_queue.get_nowait()
            except queue.Empty:
                pass

        try:
            self.playback_queue.put_nowait((event_name, sound_path))
        except queue.Full:
            self.get_logger().warn('Playback queue is full; dropping current sound event.')

    def _playback_worker(self) -> None:
        while rclpy.ok():
            try:
                event_name, sound_path = self.playback_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            try:
                subprocess.run(
                    [self.aplay_binary, sound_path],
                    check=False,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                self.get_logger().info(f'Played sound for event: {event_name}')
            except Exception as exc:
                self.get_logger().error(f'Failed to play sound {sound_path}: {exc}')


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
