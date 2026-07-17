#!/usr/bin/env python3
"""
Command Router Node
===================
Deterministic replacement for the LLM orchestrator. Same seam, no model in the
control path:

  /user_speech (String)
    -> IntentParser (verb lexicon + fuzzy synonym match, offline)
       -> LLMIntentBackend fallback (ONE stateless call, only when the parser
          cannot resolve the utterance and the fallback is enabled)
    -> Intent{action, tool_class}
    -> guards checked against /get_world_model (perception ground truth)
    -> the same skill actions the LLM used to call
    -> terse template responses on /system_response

What changed and why: the LLM used to decide the SEQUENCE of robot actions and
narrate state from its own beliefs. That produced pick-place-pick detours,
"already handed over" fictions, and synonym refusals — all control-flow
failures. Control flow is now code: every guard reads the live world model
(gripper_holds_tool is gripper ground truth), every response template states
what the code just did, and the interaction FSM makes illegal sequences
unrepresentable. The LLM keeps the one job it is good at: understanding odd
phrasing, as a fallback classifier in llm_intent_backend.py.

Parameters:
  fuzzy_threshold (float)     confident synonym-match score, default 0.8
  fuzzy_floor (float)         below this a match is no evidence, default 0.6
  llm_fallback_enabled (bool) allow the stateless LLM intent fallback
  openai_api_key (str)        key for the fallback (or OPENAI_API_KEY env)
  model_name (str)            fallback model, default gpt-4o-mini
  action_timeout_sec (float)  per-action wait cap, default 120
"""

import json
import os
import threading

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import String

from tracking_msgs.action import (
    HandoverTool,
    ReturnToolHome, PickTool, ReleaseTool, ReturnHome, ReturnTool)
from std_srvs.srv import Trigger
from tracking_msgs.srv import CountInstruments, GetWorldModel

from intent_parser import Action, Intent, IntentParser
from llm_intent_backend import LLMIntentBackend


# ── Interaction FSM ─────────────────────────────────────────────────
# Coarse states of the surgeon-robot interaction, for guards and logging.
# The gripper/tray facts always come fresh from /get_world_model — the FSM
# never claims to know more than perception does.
IDLE = 'IDLE'              # arm home, gripper empty
PICKING = 'PICKING'        # pick_tool in flight
PRESENTING = 'PRESENTING'  # holding a tool, handover in flight
RETURNING = 'RETURNING'    # return_tool / put_back motion in flight


class CommandRouterNode(Node):
    def __init__(self):
        super().__init__('command_router_node')

        self.declare_parameter('fuzzy_threshold', 0.8)
        self.declare_parameter('fuzzy_floor', 0.6)
        self.declare_parameter('llm_fallback_enabled', True)
        self.declare_parameter('openai_api_key', '')
        self.declare_parameter('model_name', 'gpt-4o-mini')
        self.declare_parameter('action_timeout_sec', 120.0)

        self.action_timeout_sec = float(
            self.get_parameter('action_timeout_sec').value)

        share = get_package_share_directory('tracking_pkg')
        self.catalog = self._load_yaml(
            os.path.join(share, 'config', 'tool_catalog.yaml'),
            key='catalog')
        lexicon = self._load_yaml(
            os.path.join(share, 'config', 'command_lexicon.yaml'))

        self.parser = IntentParser(
            self.catalog, lexicon,
            fuzzy_threshold=float(self.get_parameter('fuzzy_threshold').value),
            fuzzy_floor=float(self.get_parameter('fuzzy_floor').value))

        self._llm = None
        if bool(self.get_parameter('llm_fallback_enabled').value):
            api_key = (self.get_parameter('openai_api_key').value
                       or os.environ.get('OPENAI_API_KEY', ''))
            if api_key:
                self._llm = LLMIntentBackend(
                    api_key, self.get_parameter('model_name').value,
                    self.catalog, logger=self.get_logger())
            else:
                self.get_logger().warn(
                    'llm_fallback_enabled but no API key — running fully '
                    'deterministic')

        # One command at a time; concurrent speech is dropped, like before.
        self._busy = threading.Lock()
        self._fsm = IDLE
        self._active_goal_handle = None
        # Handover runs asynchronously so the router stays free to take new
        # commands ("wrong tool", "stop") while the robot waits for the
        # surgeon's gesture. _pending_handover holds that in-flight goal.
        self._pending_lock = threading.Lock()
        self._pending_handover = None
        # Tool class of the in-flight handover, so a tool_lost failure can
        # drive a deterministic retry of the same instrument.
        self._active_handover_class = ''
        self._handover_retry_count = 0
        self._max_handover_retries = 2
        # Pick-stage retries within one command (executor missed the grasp).
        self._max_pick_retries = 2

        # ── ROS interfaces (same seam as the LLM orchestrator) ───
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
        self.return_client = ActionClient(
            self, ReturnTool, 'return_tool', callback_group=cb)
        self.put_back_client = ActionClient(
            self, ReturnToolHome, 'return_tool_home', callback_group=cb)
        self.register_client = self.create_client(
            Trigger, '/register_inventory', callback_group=cb)
        self.count_client = self.create_client(
            CountInstruments, '/count_instruments', callback_group=cb)

        self.create_subscription(
            String, '/user_speech', self._speech_cb, 10, callback_group=cb)
        self.response_pub = self.create_publisher(String, '/system_response', 10)

        self.get_logger().info(
            f'CommandRouterNode ready ({len(self.catalog)} catalog classes, '
            f'llm_fallback={"on" if self._llm else "off"})')

    # ── Loaders ─────────────────────────────────────────────────────

    def _load_yaml(self, path, key=None):
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f) or {}
            return (data.get(key, {}) or {}) if key else data
        except Exception as e:
            self.get_logger().warn(f'Could not load {path}: {e}')
            return {}

    # ── FSM ─────────────────────────────────────────────────────────

    def _set_fsm(self, state):
        if state != self._fsm:
            self.get_logger().info(f'FSM: {self._fsm} -> {state}')
            self._fsm = state

    # ── Speech entry point ──────────────────────────────────────────

    def _speech_cb(self, msg):
        text = msg.data.strip()
        if not text:
            return
        if not self._busy.acquire(blocking=False):
            self.get_logger().warn(f'Busy — dropping speech: "{text}"')
            return
        self.get_logger().info(f'User speech: "{text}"')
        self._handover_retry_count = 0  # fresh command resets retries
        threading.Thread(
            target=self._command_thread, args=(text,), daemon=True).start()

    def _command_thread(self, text):
        try:
            self._handle_text(text)
        except Exception as e:
            self.get_logger().error(f'Command failed: {e}')
            self._publish_response(f'System error: {e}')
        finally:
            self._busy.release()

    # ── Intent resolution ───────────────────────────────────────────

    def _handle_text(self, text):
        intent = self.parser.parse(text)
        if intent.action == Action.UNKNOWN and not intent.candidates \
                and self._llm is not None:
            res = self._llm.classify(text)
            if res is not None:
                action_str, tool_class = res
                intent = Intent(Action(action_str), tool_class=tool_class,
                                raw=text, source='llm')
        self.get_logger().info(
            f'Intent: {intent.action.value}'
            f'({intent.tool_class or ",".join(intent.candidates) or "-"}) '
            f'[{intent.source}, score={intent.score:.2f}]')
        self._dispatch(intent)

    def _dispatch(self, intent):
        a = intent.action
        if a == Action.PICK:
            return self._cmd_pick(intent)
        if a == Action.RETURN:
            return self._cmd_return()
        if a == Action.PUT_BACK:
            return self._cmd_put_back(intent)
        if a == Action.RELEASE:
            return self._cmd_simple(self.release_client, ReleaseTool.Goal(),
                                    'release_tool', 'Released.')
        if a == Action.HOME:
            return self._cmd_simple(self.home_client, ReturnHome.Goal(),
                                    'return_home', 'Home.')
        if a == Action.ABORT:
            return self._cmd_abort()
        if a == Action.REGISTER:
            return self._cmd_register()
        if a == Action.COUNT:
            return self._cmd_count()
        self._publish_response("Didn't catch that.")

    # ── Command handlers ────────────────────────────────────────────

    def _cmd_pick(self, intent):
        if not intent.tool_class and not intent.candidates:
            self._publish_response('Which instrument?')
            return

        wm = self._world_model()
        if wm is None:
            self._publish_response('Cannot see tray.')
            return

        # Guard: never pick while holding — picking opens the gripper and
        # would drop the held tool. This kills the pick-place-pick detours.
        with self._pending_lock:
            presenting = self._pending_handover is not None
        if wm.get('gripper_holds_tool') or presenting:
            held = self._display_name(wm.get('active_tool_class') or '') \
                or 'tool'
            self._publish_response(
                f'Still holding {held}. Say wrong to return it.')
            return

        tool_class = intent.tool_class
        if not tool_class:
            # Ambiguous by catalog ("scissors" — long or short?), but maybe
            # not by tray: when only ONE of the candidate classes is actually
            # lying there, that is the one the surgeon means. Only ask when
            # the tray itself offers a real choice.
            tool_class = self._disambiguate_pick(wm, intent.candidates)
            if tool_class is None:
                return  # _disambiguate_pick already answered

        name = self._display_name(tool_class)
        tool_id = self._resolve_tool_id(wm, tool_class)
        if tool_id is None:
            # Not pickable — say precisely why, from ground truth.
            if any(t.get('class') == tool_class
                   for t in wm.get('reclaim_tools', [])):
                # Rule of the reclaim tray: used tools are never handed over.
                self._publish_response(f'{name} on reclaim tray.')
                return
            states = self._inventory_states(wm, tool_class)
            if states and all(s == 'IN_USE' for s in states):
                self._publish_response(f'{name} already with you.')
                return
            self._publish_response(f'No {name.lower()} on tray.')
            return

        self._publish_response(f'{name}. Picking.')
        self._do_pick_and_handover(tool_id, tool_class)

    def _disambiguate_pick(self, wm, candidates):
        """Resolve catalog-ambiguous candidates against the tray. Returns the
        single resolved class, or None after publishing the answer itself."""
        avail = {t.get('class') for t in (wm.get('available_tools') or [])}
        present = [c for c in candidates if c in avail]
        if len(present) == 1:
            self.get_logger().info(
                f'Ambiguity resolved by tray: {candidates} -> {present[0]}')
            return present[0]
        if len(present) > 1:
            names = [self._display_name(c) for c in present]
            self._publish_response(f'Which one? {" or ".join(names)}?')
            return None
        # None on the instrument tray: the reclaim tray or the surgeon has it.
        reclaim = {t.get('class') for t in (wm.get('reclaim_tools') or [])}
        on_rec = [c for c in candidates if c in reclaim]
        if len(on_rec) == 1:
            self._publish_response(
                f'{self._display_name(on_rec[0])} on reclaim tray.')
            return None
        in_use = [c for c in candidates
                  if (s := self._inventory_states(wm, c))
                  and all(x == 'IN_USE' for x in s)]
        if len(in_use) == len(candidates) and in_use:
            self._publish_response(
                f'{self._display_name(in_use[0])} already with you.')
            return None
        names = ' or '.join(self._display_name(c).lower() for c in candidates)
        self._publish_response(f'No {names} on tray.')
        return None

    def _do_pick_and_handover(self, tool_id, tool_class):
        """Pick (with deterministic re-tries on a missed grasp), then start the
        handover asynchronously — it blocks on the surgeon's gesture, and the
        router must stay commandable ("stop", "wrong") while the arm waits."""
        name = self._display_name(tool_class)
        self._set_fsm(PICKING)

        pick_result = None
        for attempt in range(1 + self._max_pick_retries):
            goal = PickTool.Goal()
            goal.tool_id = tool_id
            pick_result, err = self._send_action(
                self.pick_client, goal, 'pick_tool')
            if pick_result is None:
                self._set_fsm(IDLE)
                self._publish_response(f'Cannot pick. {err}')
                return
            if pick_result.success:
                break
            msg = pick_result.message or ''
            if 'already_holding' in msg:
                self._set_fsm(IDLE)
                self._publish_response('Still holding tool. Resolve it first.')
                return
            if ('grasp_failed' in msg or 'tool_lost' in msg) \
                    and attempt < self._max_pick_retries:
                # Missed grasp: the arm is home again. Re-read the tray — the
                # track id may have changed — and try the same class again.
                wm = self._world_model()
                tool_id = (self._resolve_tool_id(wm, tool_class)
                           if wm else None)
                if tool_id is None:
                    self._set_fsm(IDLE)
                    self._publish_response(
                        f'Missed. No {name.lower()} on tray now.')
                    return
                self._publish_response('Missed. Retrying.')
                continue
            self._set_fsm(IDLE)
            self._publish_response(f'Cannot pick. {msg}')
            return

        # Tool is in the gripper — hand it over, asynchronously.
        self._publish_response('waiting for handoff')
        handover_goal = HandoverTool.Goal()
        handover_goal.hand_pose = PoseStamped()  # empty -> wait for gesture
        gh, err = self._start_action(
            self.handover_client, handover_goal, 'handover_tool')
        if gh is None:
            self._set_fsm(IDLE)
            self._publish_response(f'Handover failed to start. {err}')
            return

        with self._pending_lock:
            self._pending_handover = gh
        self._active_handover_class = (
            pick_result.picked_tool_class or tool_class)
        self._set_fsm(PRESENTING)
        gh.get_result_async().add_done_callback(self._on_handover_done)

    def _on_handover_done(self, future):
        """Result callback of the async handover, on an executor thread."""
        with self._pending_lock:
            self._pending_handover = None
        cls = self._active_handover_class
        self._active_handover_class = ''
        try:
            wrapped = future.result()
        except Exception as e:
            self.get_logger().warn(f'handover result error: {e}')
            self._set_fsm(IDLE)
            return
        if (wrapped.status == GoalStatus.STATUS_SUCCEEDED
                and getattr(wrapped.result, 'success', False)):
            self._handover_retry_count = 0
            self._set_fsm(IDLE)
            self._publish_response('handoff done')
            return

        message = (getattr(wrapped.result, 'message', '') or '')
        if 'tool_lost' in message or 'grasp_failed' in message:
            # Tool slipped out mid-handover; the arm is home again. Retry the
            # same class deterministically — no LLM turn, just a re-dispatch.
            self._set_fsm(IDLE)
            if self._handover_retry_count >= self._max_handover_retries:
                self._handover_retry_count = 0
                self._publish_response('Cannot pick. Tool keeps dropping.')
                return
            self._handover_retry_count += 1
            threading.Thread(
                target=self._retry_thread, args=(cls,), daemon=True).start()
            return
        # canceled / aborted (e.g. preempted by return_tool): stay silent —
        # the preempting command reports its own result.
        self._set_fsm(IDLE)

    def _retry_thread(self, tool_class):
        if not self._busy.acquire(blocking=False):
            self.get_logger().warn('Busy — dropping handover retry')
            return
        try:
            self._publish_response('Dropped it. Retrying.')
            self._cmd_pick(Intent(Action.PICK, tool_class=tool_class,
                                  raw='(autonomous retry)'))
        except Exception as e:
            self.get_logger().error(f'Retry failed: {e}')
        finally:
            self._busy.release()

    def _cmd_return(self):
        wm = self._world_model()
        with self._pending_lock:
            presenting = self._pending_handover is not None
        holds = bool(wm and wm.get('gripper_holds_tool'))
        if not holds and not presenting:
            # Nothing in the gripper. "Put it back" with a loaded reclaim
            # tray means the single used tool there.
            if wm and wm.get('reclaim_tools'):
                return self._cmd_put_back(Intent(Action.PUT_BACK, raw=''))
            self._publish_response('Nothing held.')
            return
        self._set_fsm(RETURNING)
        # skill_executor preempts a running handover on its own.
        result, err = self._send_action(
            self.return_client, ReturnTool.Goal(), 'return_tool')
        self._set_fsm(IDLE)
        if result is None:
            self._publish_response(f'Cannot return. {err}')
        elif result.success:
            self._publish_response('Returned.')
        else:
            self._publish_response(f'Cannot return. {result.message}')

    def _cmd_put_back(self, intent):
        wm = self._world_model()
        if wm is None:
            self._publish_response('Cannot see tray.')
            return

        tool_class = intent.tool_class
        if not tool_class and intent.candidates:
            # Ambiguous by catalog — but only what actually lies on the
            # reclaim tray can be put back, so let the tray decide.
            reclaim_classes = {t.get('class')
                               for t in (wm.get('reclaim_tools') or [])}
            present = [c for c in intent.candidates if c in reclaim_classes]
            if len(present) == 1:
                tool_class = present[0]
            elif len(present) > 1:
                names = [self._display_name(c) for c in present]
                self._publish_response(f'Which one? {" or ".join(names)}?')
                return
            else:
                self._publish_response('Nothing of that on reclaim tray.')
                return

        # "Put the awl back" while the awl is in the gripper is a RETURN.
        if wm.get('gripper_holds_tool') and (
                not tool_class
                or tool_class == wm.get('active_tool_class')):
            return self._cmd_return()

        reclaim = wm.get('reclaim_tools') or []
        if not reclaim:
            self._publish_response('Nothing on reclaim tray.')
            return
        match = self._match_reclaim_tool(tool_class or '', reclaim)
        if match is None:
            there = ', '.join(t.get('display_name') or t.get('class', '?')
                              for t in reclaim)
            self._publish_response(f'Not on reclaim tray. There: {there}.')
            return

        goal = ReturnToolHome.Goal()
        # The class, not the track id — it survives the tool being occluded
        # between now and the moment the arm actually grasps it.
        goal.tool_id = match['class']
        # Async, like the handover — the surgeon must be able to stop a
        # moving arm by voice while this runs.
        gh, err = self._start_action(
            self.put_back_client, goal, 'return_tool_home')
        if gh is None:
            self._publish_response(f'Cannot put back. {err}')
            return
        self._set_fsm(RETURNING)
        gh.get_result_async().add_done_callback(self._on_put_back_done)
        name = match.get('display_name') or match['class']
        self._publish_response(f'Putting back {name}.')

    @staticmethod
    def _match_reclaim_tool(name, reclaim):
        """Find the reclaim-tray tool the surgeon means. Empty name is only
        unambiguous when exactly one tool lies there."""
        q = (name or '').strip().lower()
        if not q:
            return reclaim[0] if len(reclaim) == 1 else None
        for t in reclaim:
            if q in (str(t.get('id', '')).lower(),
                     str(t.get('class', '')).lower(),
                     str(t.get('display_name', '')).lower()):
                return t
        return None

    def _on_put_back_done(self, future):
        self._set_fsm(IDLE)
        try:
            wrapped = future.result()
        except Exception as e:
            self.get_logger().warn(f'put_back result error: {e}')
            return
        res = wrapped.result
        if getattr(res, 'success', False):
            n = len(res.returned_slot_ids)
            self._publish_response(f'{n} back' if n != 1 else 'Back.')
        else:
            skipped = list(getattr(res, 'skipped_reasons', []) or [])
            reason = skipped[0] if skipped else getattr(res, 'message', 'failed')
            self._publish_response(f'Cannot put back. {reason}')

    def _cmd_simple(self, client, goal, label, ok_text):
        result, err = self._send_action(client, goal, label)
        if result is None:
            self._publish_response(f'{label} failed. {err}')
        elif result.success:
            self._set_fsm(IDLE)
            self._publish_response(ok_text)
        else:
            self._publish_response(f'{label} failed. {result.message}')

    def _cmd_abort(self):
        with self._pending_lock:
            gh = self._pending_handover
        if gh is None:
            gh = self._active_goal_handle
        if gh is None:
            self._publish_response('Nothing to stop.')
            return
        try:
            cancel_future = gh.cancel_goal_async()
            self._wait_for_future(cancel_future, 5.0)
            self._set_fsm(IDLE)
            self._publish_response('Stopped.')
        except Exception as e:
            self._publish_response(f'Cannot stop. {e}')

    def _cmd_register(self):
        if not self.register_client.wait_for_service(timeout_sec=5.0):
            self._publish_response('Inventory service unavailable.')
            return
        fut = self.register_client.call_async(Trigger.Request())
        resp = self._wait_for_future(fut, 10.0)
        if resp is None:
            self._publish_response('Register timed out.')
            return
        if not resp.success:
            # e.g. two identical tools too close together — read the refusal out.
            self._publish_response(f'Cannot register. {resp.message}')
            return
        try:
            payload = json.loads(resp.message)
            found = payload.get('found', {}) or {}
            total = payload.get('total', sum(found.values()))
        except Exception:
            self._publish_response('Registered.')
            return
        parts = []
        for cls, n in sorted(found.items()):
            label = self._display_name(cls)
            parts.append(label if n == 1 else f'{n}x {label}')
        self._publish_response(f'Registered {total}. {", ".join(parts)}.')

    def _cmd_count(self):
        if not self.count_client.wait_for_service(timeout_sec=5.0):
            self._publish_response('Count service unavailable.')
            return
        fut = self.count_client.call_async(CountInstruments.Request())
        resp = self._wait_for_future(fut, 10.0)
        if resp is None:
            self._publish_response('Count timed out.')
            return
        if not resp.registered:
            self._publish_response('No operation registered.')
            return
        if resp.all_accounted_for:
            self._publish_response(f'All {resp.expected} accounted for.')
            return
        # The ones that matter: nowhere visible — possibly inside the patient.
        missing = list(resp.in_use) + list(resp.unknown)
        names = ', '.join(self._slot_display_name(s) for s in missing)
        parts = [f'{len(missing)} missing. {names}.'] if missing else []
        if resp.on_reclaim:
            parts.append(f'{len(resp.on_reclaim)} on reclaim tray.')
        if resp.in_gripper:
            parts.append('One in gripper.')
        self._publish_response(' '.join(parts)
                               or f'{resp.at_home} of {resp.expected} home.')

    # ── World-model helpers ─────────────────────────────────────────

    def _world_model(self):
        """Live scene as a dict, or None. Every guard reads THIS, never a
        belief."""
        if not self.world_model_client.wait_for_service(timeout_sec=3.0):
            return None
        fut = self.world_model_client.call_async(GetWorldModel.Request())
        resp = self._wait_for_future(fut, 5.0)
        if resp is None or not resp.success:
            return None
        try:
            return json.loads(resp.world_model_json)
        except Exception:
            return None

    @staticmethod
    def _resolve_tool_id(wm, tool_class):
        """Class -> highest-confidence track id on the instrument tray."""
        cands = [t for t in (wm.get('available_tools') or [])
                 if t.get('class') == tool_class]
        if not cands:
            return None
        best = max(cands, key=lambda t: float(t.get('confidence', 0.0)))
        return best.get('id') or None

    @staticmethod
    def _inventory_states(wm, tool_class):
        inv = (wm.get('inventory') or {}).get('tools', []) or []
        return [t.get('state') for t in inv if t.get('class') == tool_class]

    def _display_name(self, tool_class):
        info = self.catalog.get(tool_class or '', {})
        return info.get('display_name') or (tool_class or '')

    def _slot_display_name(self, slot_id):
        # slot ids look like 'awl_1' / 'scissors_long_2' — strip the instance
        # suffix and resolve the class to its display name.
        cls = slot_id.rsplit('_', 1)[0] if '_' in slot_id else slot_id
        return self._display_name(cls) or slot_id

    # ── Action / future helpers ─────────────────────────────────────

    def _start_action(self, client, goal, label):
        """Send a goal and wait until it is accepted (not for the result).
        Returns (goal_handle, None) or (None, error)."""
        if not client.wait_for_server(timeout_sec=5.0):
            return None, f'{label}: action server unavailable'
        send_future = client.send_goal_async(goal)
        gh = self._wait_for_future(send_future, 10.0)
        if gh is None:
            return None, f'{label}: goal-send timed out'
        if not gh.accepted:
            return None, f'{label}: goal rejected by server'
        return gh, None

    def _send_action(self, client, goal, label):
        """Send a goal and block until the result. Returns (result, None)
        or (None, error)."""
        gh, err = self._start_action(client, goal, label)
        if gh is None:
            return None, err
        self._active_goal_handle = gh
        result_future = gh.get_result_async()
        wrapped = self._wait_for_future(result_future, self.action_timeout_sec)
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
    node = CommandRouterNode()
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
