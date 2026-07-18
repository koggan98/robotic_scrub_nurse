#!/usr/bin/env python3
"""
HRI Display Node — a surgeon-facing traffic light
=================================================
A minimal, standalone window (drag it in front of RViz) that turns the robot's
live ROS state into a 3-colour traffic light plus one line of text. Pure
subscriber — it never touches the control path.

Colour semantics (human-action-centric):
  RED    — hands off: the arm is moving, or an error. Do not reach in.
  AMBER  — your turn: the robot is listening for your command, or waiting for
           your hand-over gesture.
  GREEN  — safe / go: idle & ready, or the tool is presented — take it.

Design rules (HRI best practice):
  * Steady for ongoing states; blink ONLY for transient alerts (bounded time),
    then revert to the live state — flashing reserved for "look now".
  * Optional slow "breathing" pulse on the green TAKE state as an invitation.
  * Never colour alone: a big headline word + a text line back up the colour
    (red-green colour-vision deficiency), with colour-blind-friendly hues.
  * Debounce so quick state transitions don't strobe; RED/alerts commit at once.

Subscribes (all already on the bus):
  /system_state_update (String  "state:tool_id:tool_class")
  /system_response     (String)      terse human line
  /handover_waiting    (Bool)        waiting for the gesture
  /hand_state          (HandState)   surgeon's hand tracked
  /handover_event      (String)      gesture_detected / reachability:unreachable
  /asr_status          (String)      'listening' (armed) / '' (idle)
"""

import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

from tracking_msgs.msg import HandState

from hri_display_logic import (
    Debouncer, is_alert_response, resolve_display, wrap_lines)


# ── Colour-blind-friendly hues (hex), converted to BGR at load ──────
def _bgr(hex_str):
    h = hex_str.lstrip('#')
    r, g, b = int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)
    return (b, g, r)


class HriDisplayNode(Node):
    def __init__(self):
        super().__init__('hri_display_node')

        self.declare_parameter('window_name', 'Scrub Nurse')
        self.declare_parameter('canvas_width', 960)
        self.declare_parameter('canvas_height', 600)
        self.declare_parameter('window_x', 40)
        self.declare_parameter('window_y', 40)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('alert_flash_sec', 2.5)
        self.declare_parameter('blink_hz', 2.0)
        self.declare_parameter('pulse_take_tool', True)
        # Executor state changes are deliberate transitions. Keep only a short
        # debounce against visual flicker so amber/green feedback stays prompt;
        # safety-relevant red states and alerts remain immediate in Debouncer.
        self.declare_parameter('debounce_sec', 0.1)
        self.declare_parameter('color_red', '#E03B24')
        self.declare_parameter('color_amber', '#F5A623')
        self.declare_parameter('color_green', '#2FB170')
        self.declare_parameter('color_boot', '#6E7681')   # neutral grey
        self.declare_parameter('color_bg', '#141414')
        self.declare_parameter('hand_conf_threshold', 0.3)
        # Show "starting up" (grey) instead of green "ready" until the ASR node
        # reports it is listening. The fallback timeout lets the display go ready
        # anyway if ASR never comes up (e.g. topic-injection testing, no mic).
        self.declare_parameter('boot_timeout_sec', 60.0)

        self.window_name = self.get_parameter('window_name').value
        self.cw = int(self.get_parameter('canvas_width').value)
        self.ch = int(self.get_parameter('canvas_height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.alert_flash_sec = float(self.get_parameter('alert_flash_sec').value)
        self.blink_hz = float(self.get_parameter('blink_hz').value)
        self.pulse_take = bool(self.get_parameter('pulse_take_tool').value)
        self.debounce_sec = float(self.get_parameter('debounce_sec').value)
        self.col_red = _bgr(self.get_parameter('color_red').value)
        self.col_amber = _bgr(self.get_parameter('color_amber').value)
        self.col_green = _bgr(self.get_parameter('color_green').value)
        self.col_boot = _bgr(self.get_parameter('color_boot').value)
        self.col_bg = _bgr(self.get_parameter('color_bg').value)
        self.hand_conf_threshold = float(
            self.get_parameter('hand_conf_threshold').value)
        self.boot_timeout_sec = float(
            self.get_parameter('boot_timeout_sec').value)
        self._asr_ready = False
        self._start_time = time.time()

        # ── Live inputs ──
        self.system_state = 'IDLE'
        self.active_tool_class = ''
        self.last_response = ''
        self.handover_waiting = False
        self.asr_listening = False
        self.hand_tracked = False
        self._alert_until = 0.0
        self._alert_text = ''

        self._debouncer = Debouncer(self.debounce_sec)

        self.create_subscription(
            String, '/system_state_update', self._state_cb, 10)
        self.create_subscription(
            String, '/system_response', self._response_cb, 10)
        self.create_subscription(
            Bool, '/handover_waiting', self._waiting_cb, 10)
        self.create_subscription(
            HandState, '/hand_state', self._hand_cb, 10)
        self.create_subscription(
            String, '/handover_event', self._event_cb, 10)
        # Latched, to match the ASR node — receive the last status (incl. the
        # initial 'ready') even if the display resubscribes after ASR is up.
        self.create_subscription(
            String, '/asr_status', self._asr_cb,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, self.cw, self.ch)
        try:
            cv2.moveWindow(self.window_name,
                           int(self.get_parameter('window_x').value),
                           int(self.get_parameter('window_y').value))
        except Exception:
            pass

        self.create_timer(1.0 / max(1.0, self.fps), self._render)
        self.get_logger().info('HriDisplayNode ready.')

    # ── Subscriptions (just latch state) ────────────────────────────

    def _state_cb(self, msg):
        parts = msg.data.split(':')
        self.system_state = parts[0] if parts else 'IDLE'
        self.active_tool_class = parts[2] if len(parts) > 2 else ''

    def _response_cb(self, msg):
        text = (msg.data or '').strip()
        self.last_response = text
        if is_alert_response(text):
            self._raise_alert(text)

    def _waiting_cb(self, msg):
        self.handover_waiting = bool(msg.data)

    def _hand_cb(self, msg):
        self.hand_tracked = bool(
            msg.is_tracked and msg.confidence > self.hand_conf_threshold)

    def _event_cb(self, msg):
        if msg.data == 'reachability:unreachable':
            self._raise_alert('Gesture out of reach')

    def _asr_cb(self, msg):
        # Any status from the ASR node means it is up and listening.
        self._asr_ready = True
        self.asr_listening = (msg.data == 'listening')

    def _raise_alert(self, text):
        self._alert_until = time.time() + self.alert_flash_sec
        self._alert_text = text

    def _resolve(self):
        booted = self._asr_ready or (
            (time.time() - self._start_time) > self.boot_timeout_sec)
        return resolve_display(
            alert_active=time.time() < self._alert_until,
            alert_text=self._alert_text,
            system_state=self.system_state,
            active_tool_class=self.active_tool_class,
            last_response=self.last_response,
            handover_waiting=self.handover_waiting,
            pulse_take=self.pulse_take,
            booted=booted)

    # ── Render ──────────────────────────────────────────────────────

    def _render(self):
        resolved = self._resolve()
        color_key, headline, detail, anim = self._debouncer.commit(
            resolved, time.time())

        base = {'red': self.col_red, 'amber': self.col_amber,
                'green': self.col_green, 'boot': self.col_boot}[color_key]
        now = time.time()
        if anim == 'blink':
            # Alternate full/dim at blink_hz; the dim half stays clearly the
            # same hue (not black) so the colour reading is never lost.
            on = int(now * self.blink_hz * 2) % 2 == 0
            scale = 1.0 if on else 0.35
        elif anim == 'pulse':
            scale = 0.78 + 0.22 * (0.5 + 0.5 * math.sin(2 * math.pi * 0.5 * now))
        else:
            scale = 1.0
        color = tuple(int(c * scale) for c in base)

        img = np.empty((self.ch, self.cw, 3), dtype=np.uint8)
        img[:] = self.col_bg
        # Big colour field (leaves a dark margin so text on it stays readable).
        m = int(self.ch * 0.06)
        cv2.rectangle(img, (m, m), (self.cw - m, self.ch - m), color, -1)

        # Fonts scale with the canvas so the layout holds at any window size.
        head_scale = self.ch / 170.0
        head_thick = max(2, int(self.ch / 90))
        base_detail_scale = self.ch / 460.0
        text_col = (20, 20, 20)  # dark text on the saturated field

        self._center_text(img, headline, y=int(self.ch * 0.38),
                          scale=head_scale, thickness=head_thick, color=text_col)

        # Detail: wrap over as many lines as needed, then shrink to fit the band
        # below the headline — a long register read-back shows in full instead of
        # being cut off (the "…" that used to render as "???").
        max_chars = max(8, int(self.cw / (base_detail_scale * 19)))
        lines = wrap_lines(detail, max_chars, max_lines=6)
        if lines:
            band_top, band_bot = int(self.ch * 0.50), int(self.ch * 0.93)
            n = len(lines)
            line_h = min((band_bot - band_top) / n, self.ch * 0.12)
            detail_scale = min(base_detail_scale, line_h / 34.0)
            detail_thick = max(1, int(detail_scale * 2))
            y_start = band_top + (band_bot - band_top - line_h * n) / 2.0
            for i, line in enumerate(lines):
                self._center_text(img, line,
                                  y=int(y_start + line_h * (i + 0.72)),
                                  scale=detail_scale, thickness=detail_thick,
                                  color=text_col)

        # Listening badge: drawn ON TOP of any state so the wake-word feedback
        # is visible even while a red RECOVERY/BUSY field owns the main colour —
        # the surgeon can still see that "robot" was heard.
        if self.asr_listening:
            self._draw_listening_badge(img)

        cv2.imshow(self.window_name, img)
        cv2.waitKey(1)

    def _draw_listening_badge(self, img):
        """A small amber 'SPEAK' chip in the top-right corner, overlaid on top
        of whatever the main field shows."""
        bh = max(24, int(self.ch * 0.11))          # badge height
        pad = int(self.ch * 0.03)
        scale = bh / 34.0
        thick = max(1, int(scale * 2))
        label = 'SPEAK'
        font = cv2.FONT_HERSHEY_SIMPLEX
        (tw, th), _ = cv2.getTextSize(label, font, scale, thick)
        dot_r = bh // 4
        inner_pad = int(bh * 0.35)
        bw = inner_pad + 2 * dot_r + inner_pad // 2 + tw + inner_pad
        x1, y1 = self.cw - pad - bw, pad
        x2, y2 = self.cw - pad, pad + bh
        cv2.rectangle(img, (x1, y1), (x2, y2), self.col_amber, -1)
        dark = (20, 20, 20)
        cx = x1 + inner_pad + dot_r
        cy = (y1 + y2) // 2
        cv2.circle(img, (cx, cy), dot_r, dark, -1)
        cv2.putText(img, label,
                    (cx + dot_r + inner_pad // 2, cy + th // 2),
                    font, scale, dark, thick, cv2.LINE_AA)

    def _center_text(self, img, text, y, scale, thickness, color):
        if not text:
            return
        font = cv2.FONT_HERSHEY_SIMPLEX
        (tw, _), _ = cv2.getTextSize(text, font, scale, thickness)
        x = max(6, (self.cw - tw) // 2)
        cv2.putText(img, text, (x, y), font, scale, color, thickness,
                    cv2.LINE_AA)

    def destroy_node(self):
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HriDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
