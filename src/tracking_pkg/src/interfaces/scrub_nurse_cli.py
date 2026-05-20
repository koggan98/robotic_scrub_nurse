#!/usr/bin/env python3
"""
Scrub Nurse CLI
===============
Interactive REPL for the LLM scrub nurse: shows the current world model
snapshot and lets you trigger the four skill_executor actions
(pick_tool, handover_tool, release_tool, return_home) with a single
keystroke instead of typing full `ros2 action send_goal` commands.

Run after sourcing the workspace:
    source /home/mir/robotic_scrub_nurse_ws/install/setup.bash
    python3 ros_unrelated_scripts/scrub_nurse_cli.py
"""

import json
import sys
import threading
import time
from datetime import datetime

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

from tracking_pkg.action import (
    HandoverTool,
    PickTool,
    ReleaseTool,
    ReturnHome,
    ReturnTool,
)
from tracking_pkg.srv import GetWorldModel


HELP_TEXT = """\
Commands:
  <Enter> / s    refresh world-model snapshot
  p              pick highest-confidence tool
  p N            pick tool at list index N
  p <tool_id>    pick by persistent id (e.g. 'p tool_3')
  h              handover (skill_executor uses last /hand_state)
  b              bring picked tool back to its pickup spot
  r              release gripper now
  H              return home
  ? / help       this help
  q / quit       exit
"""


class ScrubNurseCLI(Node):
    def __init__(self):
        super().__init__('scrub_nurse_cli')

        self.world_model_client = self.create_client(
            GetWorldModel, '/get_world_model')
        self.pick_client = ActionClient(self, PickTool, 'pick_tool')
        self.handover_client = ActionClient(self, HandoverTool, 'handover_tool')
        self.release_client = ActionClient(self, ReleaseTool, 'release_tool')
        self.home_client = ActionClient(self, ReturnHome, 'return_home')
        self.return_client = ActionClient(self, ReturnTool, 'return_tool')

        self._cancel_requested = threading.Event()

        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self)

    # ── Snapshot / status ──────────────────────────────────────────

    def refresh_snapshot(self, timeout_s=3.0):
        if not self.world_model_client.wait_for_service(timeout_sec=1.0):
            return None
        req = GetWorldModel.Request()
        future = self.world_model_client.call_async(req)
        deadline = time.time() + timeout_s
        while rclpy.ok() and not future.done() and time.time() < deadline:
            self._exec.spin_once(timeout_sec=0.05)
        if not future.done():
            return None
        resp = future.result()
        if not resp or not resp.success:
            return None
        try:
            return json.loads(resp.world_model_json)
        except Exception as e:
            print(f'[ERR] could not parse world_model_json: {e}')
            return None

    @staticmethod
    def render_status(snap):
        if snap is None:
            print('  (no snapshot — is the launch running?)')
            return

        ts = datetime.now().strftime('%H:%M:%S')
        print(f'─── Scrub Nurse @ {ts} '.ljust(60, '─'))
        sys_state = snap.get('system_state', '?')
        active_id = snap.get('active_tool_id') or '-'
        active_cls = snap.get('active_tool_class') or ''
        active = f'{active_id} ({active_cls})' if active_cls else active_id
        print(f'system_state : {sys_state:<10}  active_tool : {active}')
        print(f'robot_ready  : {snap.get("robot_ready", False)}')

        g = snap.get('last_gesture') or {}
        gname = g.get('name', '') or '(none)'
        gage = g.get('age_sec')
        gfresh = g.get('fresh', False)
        if gname == '(none)':
            print('last_gesture : (none)')
        else:
            age_str = f'{gage:.1f}s' if gage is not None else '?'
            print(f'last_gesture : {gname}  age={age_str}  fresh={gfresh}')

        hand = snap.get('receiver_hand') or {}
        if hand.get('detected'):
            pose = hand.get('pose') or [0, 0, 0]
            conf = hand.get('confidence', 0.0)
            print(f'receiver_hand: detected=True  conf={conf:.2f}  '
                  f'pose=({pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f})')
        else:
            print('receiver_hand: detected=False')

        tools = snap.get('available_tools') or []
        print()
        print(f'Available tools ({len(tools)}):')
        if not tools:
            print('  (none)')
        for i, t in enumerate(tools):
            gp = t.get('grasp_point') or [0, 0, 0]
            print(f'  [{i}] {t.get("id",""):<10} {t.get("class",""):<18} '
                  f'conf={t.get("confidence",0.0):.2f}  '
                  f'pos=({gp[0]:.3f}, {gp[1]:.3f})')
        print()

    # ── Action helpers ─────────────────────────────────────────────

    def _wait_until(self, future, label='running'):
        """Spin the executor until the future is done. Ctrl-C cancels."""
        self._cancel_requested.clear()
        spinner = ['|', '/', '-', '\\']
        i = 0
        try:
            while rclpy.ok() and not future.done():
                self._exec.spin_once(timeout_sec=0.1)
                sys.stdout.write(f'\r  [{label}] {spinner[i % 4]} ')
                sys.stdout.flush()
                i += 1
                if self._cancel_requested.is_set():
                    break
        except KeyboardInterrupt:
            self._cancel_requested.set()
        finally:
            sys.stdout.write('\r' + ' ' * 40 + '\r')
            sys.stdout.flush()
        return future.done()

    def _send_action(self, label, client, goal_msg):
        if not client.wait_for_server(timeout_sec=2.0):
            print(f'[{label}] action server not available')
            return False

        print(f'[{label}] sending goal …')
        send_future = client.send_goal_async(goal_msg)
        if not self._wait_until(send_future, f'{label} accept'):
            print(f'[{label}] goal-send aborted')
            return False
        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            print(f'[{label}] goal rejected by server')
            return False

        result_future = goal_handle.get_result_async()
        if not self._wait_until(result_future, f'{label} exec'):
            # User pressed Ctrl-C — try to cancel
            print(f'[{label}] cancelling …')
            cancel_future = goal_handle.cancel_goal_async()
            self._wait_until(cancel_future, f'{label} cancel')
            return False

        wrapped = result_future.result()
        if wrapped is None:
            print(f'[{label}] no result returned')
            return False
        result = wrapped.result
        ok = getattr(result, 'success', False)
        msg = getattr(result, 'message', '')
        if ok:
            extra = ''
            if hasattr(result, 'picked_tool_id') and result.picked_tool_id:
                extra = f'  picked {result.picked_tool_id} ' \
                        f'({getattr(result, "picked_tool_class", "")})'
            print(f'[{label}] SUCCESS{extra}  message: {msg}')
        else:
            print(f'[{label}] FAILED  message: {msg}')
        return ok

    # ── Skill triggers ─────────────────────────────────────────────

    def send_pick(self, tool_id=''):
        goal = PickTool.Goal()
        goal.tool_id = tool_id
        label = 'PICK' if not tool_id else f'PICK[{tool_id}]'
        self._send_action(label, self.pick_client, goal)

    def send_handover(self):
        goal = HandoverTool.Goal()
        # Empty header.frame_id → skill_executor falls back to last /hand_state
        goal.hand_pose = PoseStamped()
        self._send_action('HANDOVER', self.handover_client, goal)

    def send_return(self):
        goal = ReturnTool.Goal()
        self._send_action('RETURN', self.return_client, goal)

    def send_release(self):
        goal = ReleaseTool.Goal()
        self._send_action('RELEASE', self.release_client, goal)

    def send_home(self):
        goal = ReturnHome.Goal()
        self._send_action('HOME', self.home_client, goal)

    # ── REPL ───────────────────────────────────────────────────────

    def repl(self):
        print('Scrub Nurse CLI — type "?" for help, "q" to quit.\n')
        snap = self.refresh_snapshot()
        self.render_status(snap)

        while rclpy.ok():
            try:
                raw = input('> ').strip()
            except (EOFError, KeyboardInterrupt):
                print()
                return

            if raw in ('q', 'quit', 'exit'):
                return
            if raw in ('?', 'help'):
                print(HELP_TEXT)
                continue
            if raw == '' or raw == 's':
                snap = self.refresh_snapshot()
                self.render_status(snap)
                continue
            if raw == 'h':
                self.send_handover()
                continue
            if raw == 'b':
                self.send_return()
                continue
            if raw == 'r':
                self.send_release()
                continue
            if raw == 'H':
                self.send_home()
                continue

            if raw == 'p' or raw.startswith('p '):
                arg = raw[2:].strip() if raw.startswith('p ') else ''
                tool_id = self._resolve_pick_target(arg, snap)
                if tool_id is None:
                    continue  # error already printed
                self.send_pick(tool_id)
                continue

            print(f'unknown command "{raw}". Type "?" for help.')

    def _resolve_pick_target(self, arg, snap):
        """Resolve an empty/index/tool_id argument to a tool_id string.

        Returns the string to send (may be empty for top-confidence pick),
        or None on error.
        """
        if arg == '':
            return ''  # skill_executor picks top confidence
        # Numeric index into the snapshot list?
        if arg.isdigit():
            if snap is None:
                print('[ERR] no snapshot available — refresh first with <Enter>')
                return None
            tools = snap.get('available_tools') or []
            idx = int(arg)
            if not (0 <= idx < len(tools)):
                print(f'[ERR] index {idx} out of range (have {len(tools)} tools)')
                return None
            return tools[idx].get('id', '')
        # Otherwise treat as literal tool_id
        return arg


def main():
    rclpy.init()
    cli = ScrubNurseCLI()
    try:
        cli.repl()
    finally:
        cli._exec.shutdown()
        cli.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
