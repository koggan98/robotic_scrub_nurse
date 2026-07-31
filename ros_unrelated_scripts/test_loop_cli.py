#!/usr/bin/env python3
"""Lightweight keyboard driver for the pick -> hand over -> reclaim -> return test loop.

The normal flow needs a "start surgery" (register_inventory) once to freeze the
tray as the operation's inventory, so a returned tool knows where its home slot
is. When you keep moving a single tool to new positions for testing, that home
would be stale unless you re-register every time.

This driver does it for you: on every "go" it FIRST calls /register_inventory
(which freezes whatever is on the instrument tray right now as the home slots),
THEN issues the pick. So the tool's *current* position becomes its home and it is
returned exactly there — no "start surgery" needed.

Run it alongside the normal launch (it only publishes /user_speech, the same
channel speech uses, and calls the existing services). Type:

    <tool name>   register current layout + pick that tool  (e.g. "hammer")
    <Enter>       same, for the single tool on the tray (auto-detected)
    return / r     send the tool on the reclaim tray back to its home slot
    home / stop    park the arm / abort
    q              quit

You can still use voice at the same time — this is just an extra input.
"""

import json
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from tracking_msgs.srv import GetWorldModel


class TestLoopDriver(Node):
    def __init__(self):
        super().__init__("test_loop_driver")
        self.speech_pub = self.create_publisher(String, "/user_speech", 10)
        self.register_cli = self.create_client(Trigger, "/register_inventory")
        self.wm_cli = self.create_client(GetWorldModel, "/get_world_model")
        self.create_subscription(String, "/system_response", self._resp_cb, 10)
        self.last_tool = ""

    # ── feedback ────────────────────────────────────────────────────
    def _resp_cb(self, msg):
        print(f"\n   << {msg.data}")

    def _say(self, text):
        self.speech_pub.publish(String(data=text))
        print(f'   >> user_speech: "{text}"')

    # ── blocking service helpers (executor spins in a background thread) ──
    @staticmethod
    def _wait(future, timeout):
        ev = threading.Event()
        future.add_done_callback(lambda _f: ev.set())
        return future.result() if ev.wait(timeout) else None

    def _register(self):
        """Freeze the current instrument-tray layout as the home slots."""
        if not self.register_cli.wait_for_service(timeout_sec=5.0):
            print("   !! /register_inventory unavailable"); return False
        resp = self._wait(self.register_cli.call_async(Trigger.Request()), 10.0)
        if resp is None:
            print("   !! register timed out"); return False
        if not resp.success:
            print(f"   !! register refused: {resp.message}"); return False
        print(f"   registered current layout: {resp.message}")
        return True

    def _single_tray_tool(self):
        """Return the one instrument-tray tool class, a list if several, or None."""
        if not self.wm_cli.wait_for_service(timeout_sec=3.0):
            return None
        resp = self._wait(self.wm_cli.call_async(GetWorldModel.Request()), 5.0)
        if resp is None or not resp.success:
            return None
        try:
            wm = json.loads(resp.world_model_json)
        except (ValueError, TypeError):
            return None
        classes = sorted({t.get("class") for t in (wm.get("available_tools") or [])
                          if t.get("class")})
        return classes[0] if len(classes) == 1 else (classes or None)

    # ── actions ─────────────────────────────────────────────────────
    def pick(self, tool):
        # Register FIRST so this tool's current position becomes its home slot.
        if not self._register():
            return
        self.last_tool = tool
        self._say(tool)

    def ret(self):
        self._say(f"return {self.last_tool}" if self.last_tool else "put it back")


BANNER = """
================ pick / return test loop ================
 <tool name>  register current layout + pick (e.g. hammer)
 <Enter>      same, for the single tool on the tray
 return / r   send the reclaim-tray tool back to its home
 home         park the arm     stop  abort     q  quit
=========================================================
"""


def main():
    rclpy.init()
    node = TestLoopDriver()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    print(BANNER)
    try:
        while rclpy.ok():
            line = input("> ").strip().lower()
            if line in ("q", "quit", "exit"):
                break
            if line in ("return", "r", "back", "put back", "put it back"):
                node.ret()
            elif line in ("home", "h"):
                node._say("home")
            elif line in ("stop", "abort"):
                node._say("stop")
            elif line == "":
                t = node._single_tray_tool()
                if isinstance(t, str):
                    node.pick(t)
                elif t:
                    print(f'   ?? several tools ({", ".join(t)}) — type a name')
                else:
                    print("   ?? no tool detected on the instrument tray")
            else:
                node.pick(line)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
