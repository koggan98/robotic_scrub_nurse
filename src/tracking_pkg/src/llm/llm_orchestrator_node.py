#!/usr/bin/env python3
"""
LLM Orchestrator Node
=====================
High-level planner for the robotic scrub nurse. Receives transcribed
speech, runs an OpenAI tool-calling loop, and drives the Phase-1 skills.

Pipeline:
  /user_speech (String)
    -> OpenAI chat completion with tool calling
       tools:
         get_world_model()           -> /get_world_model service
         pick_and_handover(tool_id)  -> /pick_tool + /handover_tool actions
         release_tool()              -> /release_tool action
         return_home()               -> /return_home action
         abort()                     -> cancel running action
    -> final assistant text -> /system_response (String)

Parameters:
  openai_api_key (str)      OpenAI key (or OPENAI_API_KEY env var)
  model_name (str)          OpenAI model, default gpt-4o-mini
  knowledge_base_path (str) tool_knowledge_base.yaml for the system prompt
  max_tool_turns (int)      tool-call loop cap, default 8
  action_timeout_sec (float) per-action wait cap, default 120
"""

import json
import os
import threading

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from pydantic import BaseModel, Field
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import String

from tracking_pkg.action import HandoverTool, PickTool, ReleaseTool, ReturnHome
from tracking_pkg.srv import GetWorldModel


# ── Pydantic tool-argument models ──────────────────────────────────

class PickAndHandoverArgs(BaseModel):
    tool_id: str = Field(
        default='',
        description=(
            "Persistent tool id from the world model (e.g. 'tool_3'). "
            "Empty string = let the robot pick the highest-confidence "
            "reachable tool."
        ),
    )


_EMPTY_SCHEMA = {"type": "object", "properties": {}, "required": []}

# Sentinel for _send_action: fall back to the node's default action timeout.
_DEFAULT_TIMEOUT = object()


class LLMOrchestratorNode(Node):
    def __init__(self):
        super().__init__('llm_orchestrator_node')

        self.declare_parameter('openai_api_key', '')
        self.declare_parameter('model_name', 'gpt-4o-mini')
        self.declare_parameter('knowledge_base_path', '')
        self.declare_parameter('max_tool_turns', 8)
        self.declare_parameter('action_timeout_sec', 120.0)

        api_key = self.get_parameter('openai_api_key').value
        if not api_key:
            api_key = os.environ.get('OPENAI_API_KEY', '')
        self._api_key = api_key
        self.model_name = self.get_parameter('model_name').value
        self.max_tool_turns = int(self.get_parameter('max_tool_turns').value)
        self.action_timeout_sec = float(self.get_parameter('action_timeout_sec').value)

        kb_path = self.get_parameter('knowledge_base_path').value
        if not kb_path:
            kb_path = os.path.join(
                get_package_share_directory('tracking_pkg'),
                'config', 'tool_knowledge_base.yaml')
        self.knowledge_base = self._load_knowledge_base(kb_path)

        self._client = None  # lazy OpenAI client
        self._busy = threading.Lock()
        self._active_goal_handle = None

        # ── ROS interfaces ───────────────────────────────────────
        cb = ReentrantCallbackGroup()
        self.world_model_client = self.create_client(
            GetWorldModel, '/get_world_model', callback_group=cb)
        self.pick_client = ActionClient(
            self, PickTool, 'pick_tool', callback_group=cb)
        self.handover_client = ActionClient(
            self, HandoverTool, 'handover_tool', callback_group=cb)
        self.release_client = ActionClient(
            self, ReleaseTool, 'release_tool', callback_group=cb)
        self.home_client = ActionClient(
            self, ReturnHome, 'return_home', callback_group=cb)

        self.create_subscription(
            String, '/user_speech', self._speech_cb, 10, callback_group=cb)
        self.response_pub = self.create_publisher(String, '/system_response', 10)

        self._tool_defs = self._build_tool_defs()

        self.get_logger().info(
            f'LLMOrchestratorNode ready (model={self.model_name}, '
            f'api_key={"set" if self._api_key else "MISSING"})')

    # ── Loaders ─────────────────────────────────────────────────────

    def _load_knowledge_base(self, path):
        try:
            with open(path, 'r') as f:
                return yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().warn(f'Could not load knowledge base: {e}')
            return {}

    def _get_openai_client(self):
        if self._client is None:
            try:
                from openai import OpenAI
                self._client = OpenAI(api_key=self._api_key)
            except ImportError:
                self.get_logger().error('openai not installed: pip install openai')
                return None
            except Exception as e:
                self.get_logger().error(f'OpenAI client init failed: {e}')
                return None
        return self._client

    # ── Prompt + tool definitions ───────────────────────────────────

    def _build_system_prompt(self):
        tools_desc = ''
        for cls_name, info in self.knowledge_base.get('tools', {}).items():
            synonyms = ', '.join(info.get('synonyms', []))
            tools_desc += (
                f"- {info.get('display_name', cls_name)} "
                f"(class: {cls_name}; also called: {synonyms})\n")

        return f"""You are the reasoning core of a robotic scrub nurse in an operating room.
The surgeon speaks English. Identify the requested instrument and drive the robot.

Instruments on this system:
{tools_desc}
Functions:
- get_world_model(): tools currently on the tray (ids, classes) and system state.
- pick_and_handover(tool_id): the robot immediately picks the tool, then
  delivers it. Empty tool_id = highest-confidence reachable tool.
- release_tool(): open the gripper now.
- return_home(): move the arm to its home pose.
- abort(): cancel the running robot action.

Rules:
1. Call get_world_model() first to see the tray.
2. Match the request to exactly one tool id, then call pick_and_handover(tool_id).
   The robot picks the tool up right away. Never ask the surgeon to do
   anything, and never mention gestures or handoff signals.
3. If several tools match and you truly cannot choose, ask one short question.
4. Always reply in English, in extremely terse caveman style: drop articles
   and filler words, max ~6 words. Examples: "Needle holder. Picking." /
   "Done." / "No scalpel on tray." / "Which scissors?"
"""

    def _build_tool_defs(self):
        return [
            {"type": "function", "function": {
                "name": "get_world_model",
                "description": "Get current tray tools (ids, classes) and "
                               "system state.",
                "parameters": _EMPTY_SCHEMA}},
            {"type": "function", "function": {
                "name": "pick_and_handover",
                "description": "Robot immediately picks the tool, then "
                               "delivers it. Empty tool_id = "
                               "highest-confidence tool.",
                "parameters": PickAndHandoverArgs.model_json_schema()}},
            {"type": "function", "function": {
                "name": "release_tool",
                "description": "Open the gripper immediately.",
                "parameters": _EMPTY_SCHEMA}},
            {"type": "function", "function": {
                "name": "return_home",
                "description": "Move the arm to its home pose.",
                "parameters": _EMPTY_SCHEMA}},
            {"type": "function", "function": {
                "name": "abort",
                "description": "Cancel the currently running robot action.",
                "parameters": _EMPTY_SCHEMA}},
        ]

    # ── Speech entry point ──────────────────────────────────────────

    def _speech_cb(self, msg):
        text = msg.data.strip()
        if not text:
            return
        if not self._busy.acquire(blocking=False):
            self.get_logger().warn(f'Busy — dropping speech: "{text}"')
            return
        self.get_logger().info(f'User speech: "{text}"')
        threading.Thread(
            target=self._conversation_thread, args=(text,), daemon=True).start()

    def _conversation_thread(self, text):
        try:
            self._run_conversation(text)
        except Exception as e:
            self.get_logger().error(f'Conversation failed: {e}')
            self._publish_response(f'System error: {e}')
        finally:
            self._busy.release()

    # ── Conversation loop ───────────────────────────────────────────

    def _run_conversation(self, text):
        client = self._get_openai_client()
        if client is None:
            self._publish_response('System error: LLM not available.')
            return

        messages = [
            {"role": "system", "content": self._build_system_prompt()},
            {"role": "user", "content": text},
        ]

        for turn in range(self.max_tool_turns):
            try:
                resp = client.chat.completions.create(
                    model=self.model_name,
                    messages=messages,
                    tools=self._tool_defs,
                    max_tokens=100,
                )
            except Exception as e:
                self.get_logger().error(f'OpenAI request failed: {e}')
                self._publish_response(f'System error talking to the LLM: {e}')
                return

            m = resp.choices[0].message
            if not m.tool_calls:
                self._publish_response(m.content or '(no response)')
                return

            # Record the assistant turn (with its tool calls) verbatim.
            messages.append({
                "role": "assistant",
                "content": m.content or "",
                "tool_calls": [
                    {
                        "id": tc.id,
                        "type": "function",
                        "function": {
                            "name": tc.function.name,
                            "arguments": tc.function.arguments,
                        },
                    }
                    for tc in m.tool_calls
                ],
            })

            for tc in m.tool_calls:
                try:
                    args = json.loads(tc.function.arguments or '{}')
                except json.JSONDecodeError:
                    args = {}
                self.get_logger().info(
                    f'Tool call: {tc.function.name}({args})')
                result_str = self._dispatch_tool(tc.function.name, args)
                messages.append({
                    "role": "tool",
                    "tool_call_id": tc.id,
                    "content": result_str,
                })

        self._publish_response('Stopped: tool-call limit reached.')

    # ── Tool dispatch ───────────────────────────────────────────────

    def _dispatch_tool(self, name, args):
        if name == 'get_world_model':
            return self._tool_get_world_model()
        if name == 'pick_and_handover':
            return self._tool_pick_and_handover(args)
        if name == 'release_tool':
            return self._tool_simple_action(
                self.release_client, ReleaseTool.Goal(), 'release_tool')
        if name == 'return_home':
            return self._tool_simple_action(
                self.home_client, ReturnHome.Goal(), 'return_home')
        if name == 'abort':
            return self._tool_abort()
        return json.dumps({"success": False,
                           "message": f"unknown tool '{name}'"})

    def _tool_get_world_model(self):
        if not self.world_model_client.wait_for_service(timeout_sec=3.0):
            return json.dumps({"success": False,
                               "message": "/get_world_model unavailable"})
        future = self.world_model_client.call_async(GetWorldModel.Request())
        resp = self._wait_for_future(future, 5.0)
        if resp is None:
            return json.dumps({"success": False,
                               "message": "/get_world_model timed out"})
        if not resp.success:
            return json.dumps({"success": False, "message": resp.message})
        # world_model_json is already a JSON document — return it as-is.
        return resp.world_model_json

    def _tool_pick_and_handover(self, args):
        try:
            parsed = PickAndHandoverArgs(**args)
        except Exception as e:
            return json.dumps({"success": False,
                               "message": f"invalid arguments: {e}"})

        pick_goal = PickTool.Goal()
        pick_goal.tool_id = parsed.tool_id
        pick_result, err = self._send_action(
            self.pick_client, pick_goal, 'pick_tool')
        if pick_result is None:
            return json.dumps({"success": False, "stage": "pick",
                               "message": err})
        if not pick_result.success:
            return json.dumps({"success": False, "stage": "pick",
                               "message": pick_result.message})

        # Tool is in the gripper. The handover action below blocks until the
        # surgeon's gesture, so emit feedback now — the LLM is unreachable
        # while that call is in flight.
        self._publish_response('waiting for handoff')

        handover_goal = HandoverTool.Goal()
        # Empty pose -> skill_executor waits for the surgeon's
        # double_open_close gesture before delivering. That wait has no
        # timeout, so neither can this action call.
        handover_goal.hand_pose = PoseStamped()
        ho_result, err = self._send_action(
            self.handover_client, handover_goal, 'handover_tool',
            result_timeout=None)
        if ho_result is None:
            return json.dumps({
                "success": False, "stage": "handover", "message": err,
                "picked_tool_id": pick_result.picked_tool_id,
                "picked_tool_class": pick_result.picked_tool_class})
        if not ho_result.success:
            return json.dumps({
                "success": False, "stage": "handover",
                "message": ho_result.message,
                "picked_tool_id": pick_result.picked_tool_id,
                "picked_tool_class": pick_result.picked_tool_class})

        return json.dumps({
            "success": True,
            "picked_tool_id": pick_result.picked_tool_id,
            "picked_tool_class": pick_result.picked_tool_class,
            "message": "tool picked and handed over"})

    def _tool_simple_action(self, client, goal, label):
        result, err = self._send_action(client, goal, label)
        if result is None:
            return json.dumps({"success": False, "message": err})
        return json.dumps({"success": bool(result.success),
                           "message": result.message})

    def _tool_abort(self):
        gh = self._active_goal_handle
        if gh is None:
            return json.dumps({"success": True,
                               "message": "nothing to abort"})
        try:
            cancel_future = gh.cancel_goal_async()
            self._wait_for_future(cancel_future, 5.0)
            return json.dumps({"success": True,
                               "message": "cancel requested"})
        except Exception as e:
            return json.dumps({"success": False,
                               "message": f"abort failed: {e}"})

    # ── Action / future helpers ─────────────────────────────────────

    def _send_action(self, client, goal, label, result_timeout=_DEFAULT_TIMEOUT):
        # result_timeout: _DEFAULT_TIMEOUT -> self.action_timeout_sec,
        # None -> block indefinitely, or an explicit seconds value.
        if not client.wait_for_server(timeout_sec=5.0):
            return None, f'{label}: action server unavailable'
        send_future = client.send_goal_async(goal)
        gh = self._wait_for_future(send_future, 10.0)
        if gh is None:
            return None, f'{label}: goal-send timed out'
        if not gh.accepted:
            return None, f'{label}: goal rejected by server'
        self._active_goal_handle = gh
        if result_timeout is _DEFAULT_TIMEOUT:
            result_timeout = self.action_timeout_sec
        result_future = gh.get_result_async()
        wrapped = self._wait_for_future(result_future, result_timeout)
        self._active_goal_handle = None
        if wrapped is None:
            return None, f'{label}: result timed out'
        return wrapped.result, None

    @staticmethod
    def _wait_for_future(future, timeout):
        ev = threading.Event()
        future.add_done_callback(lambda _f: ev.set())
        if not ev.wait(timeout):
            return None
        return future.result()

    # ── Output ──────────────────────────────────────────────────────

    def _publish_response(self, text):
        msg = String()
        msg.data = text
        self.response_pub.publish(msg)
        self.get_logger().info(f'Response: "{text}"')


def main(args=None):
    rclpy.init(args=args)
    node = LLMOrchestratorNode()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
