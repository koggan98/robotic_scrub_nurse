#!/usr/bin/env python3

import argparse
import sys
import threading
import time
from collections import deque

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


DEFAULT_WRENCH_TOPIC = "/force_torque_sensor_broadcaster/wrench"


class ForceZPlotNode(Node):
    def __init__(self, topic: str, window_sec: float, zero_start: bool):
        super().__init__("force_z_live_plot")
        self.window_sec = max(1.0, float(window_sec))
        self.zero_start = zero_start
        self._lock = threading.Lock()
        self._times = deque()
        self._values = deque()
        self._start_time = None
        self._zero_offset = None
        self._last_sample_wall_time = None

        self.create_subscription(WrenchStamped, topic, self._wrench_callback, 50)
        self.get_logger().info(
            f"Plotting force.z from {topic} with {self.window_sec:.1f}s window"
        )

    def _wrench_callback(self, msg: WrenchStamped) -> None:
        now = time.monotonic()
        force_z = float(msg.wrench.force.z)

        if self.zero_start and self._zero_offset is None:
            self._zero_offset = force_z
            self.get_logger().info(
                f"Zero-start offset captured at force.z={self._zero_offset:.3f} N"
            )

        if self._zero_offset is not None:
            force_z -= self._zero_offset

        with self._lock:
            if self._start_time is None:
                self._start_time = now

            t = now - self._start_time
            self._times.append(t)
            self._values.append(force_z)
            self._last_sample_wall_time = now

            cutoff = t - self.window_sec
            while self._times and self._times[0] < cutoff:
                self._times.popleft()
                self._values.popleft()

    def snapshot(self):
        with self._lock:
            xs = list(self._times)
            ys = list(self._values)
            last_sample_wall_time = self._last_sample_wall_time

        age = None
        if last_sample_wall_time is not None:
            age = time.monotonic() - last_sample_wall_time
        return xs, ys, age


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot geometry_msgs/WrenchStamped force.z in real time."
    )
    parser.add_argument(
        "--topic",
        default=DEFAULT_WRENCH_TOPIC,
        help=f"WrenchStamped topic to subscribe to (default: {DEFAULT_WRENCH_TOPIC})",
    )
    parser.add_argument(
        "--window-sec",
        type=float,
        default=30.0,
        help="Rolling plot window in seconds (default: 30)",
    )
    parser.add_argument(
        "--zero-start",
        action="store_true",
        help="Subtract the first received force.z sample from all following samples.",
    )
    parser.add_argument(
        "--ylim",
        nargs=2,
        type=float,
        metavar=("MIN_N", "MAX_N"),
        help="Optional fixed y-axis limits in Newton.",
    )
    parser.add_argument(
        "--refresh-ms",
        type=int,
        default=100,
        help="Plot refresh interval in milliseconds (default: 100)",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print(
            "matplotlib is required for plotting. Install it, then rerun this script.",
            file=sys.stderr,
        )
        return 1

    rclpy.init()
    node = ForceZPlotNode(args.topic, args.window_sec, args.zero_start)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    plt.ion()
    fig, ax = plt.subplots()
    line, = ax.plot([], [], color="tab:blue", linewidth=1.8)
    status_text = ax.text(
        0.01,
        0.98,
        "waiting for samples...",
        transform=ax.transAxes,
        va="top",
        ha="left",
    )
    ax.set_title("TCP Force Z")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("force.z [N]" + (" relative" if args.zero_start else ""))
    ax.grid(True, alpha=0.3)
    if args.ylim:
        ax.set_ylim(args.ylim[0], args.ylim[1])

    try:
        while plt.fignum_exists(fig.number):
            xs, ys, age = node.snapshot()
            if xs:
                line.set_data(xs, ys)
                xmax = max(args.window_sec, xs[-1])
                xmin = max(0.0, xmax - args.window_sec)
                ax.set_xlim(xmin, xmax)

                if not args.ylim:
                    ymin = min(ys)
                    ymax = max(ys)
                    if ymin == ymax:
                        ymin -= 1.0
                        ymax += 1.0
                    margin = max(0.5, 0.1 * (ymax - ymin))
                    ax.set_ylim(ymin - margin, ymax + margin)

                if age is None:
                    status = f"samples: {len(ys)}"
                else:
                    status = (
                        f"latest force.z: {ys[-1]:.3f} N | "
                        f"samples: {len(ys)} | age: {age:.2f}s"
                    )
                status_text.set_text(status)

            fig.canvas.draw_idle()
            plt.pause(max(0.01, args.refresh_ms / 1000.0))
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        spin_thread.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
