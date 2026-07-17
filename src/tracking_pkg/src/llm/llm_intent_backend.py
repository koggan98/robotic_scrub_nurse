#!/usr/bin/env python3
"""LLM fallback for the hybrid NLU: one stateless call, transcript -> intent.

This is the ONLY place the LLM is allowed to live now: classifying an utterance
the deterministic parser could not resolve (odd phrasing, heavy ASR damage).
It sees no world model, calls no tools, runs no multi-turn loop — it maps text
to {"action", "tool_class"} and every output is validated against the closed
action set and the catalog before anyone acts on it. A hallucinated class dies
here, not on the robot.

The interface is deliberately tiny (classify(text) -> (action, tool_class) or
None) so a local model on the Jetson can replace OpenAI later without touching
the router.
"""

import json
from typing import Dict, Optional, Tuple

_ACTIONS = ('pick', 'return', 'put_back', 'release', 'home', 'abort',
            'register', 'count', 'unknown')


class LLMIntentBackend:
    def __init__(self, api_key: str, model_name: str, catalog: Dict,
                 logger=None, timeout_sec: float = 8.0):
        self._api_key = api_key
        self._model_name = model_name
        self._logger = logger
        self._timeout_sec = float(timeout_sec)
        self._client = None
        self._catalog = catalog or {}
        self._system_prompt = self._build_prompt()

    def _log(self, level, msg):
        if self._logger is not None:
            getattr(self._logger, level)(msg)

    def _build_prompt(self):
        tools_desc = ''
        for cls, info in self._catalog.items():
            syns = ', '.join(info.get('synonyms', []) or [])
            tools_desc += (f"- {cls} ({info.get('display_name', cls)}; "
                           f"also called: {syns})\n")
        return f"""You classify one utterance of a surgeon into a robot command.
Output ONLY a JSON object: {{"action": <action>, "tool_class": <class or null>}}

Actions:
- "pick": hand the named instrument to the surgeon
- "return": the tool the robot holds is wrong, bring it back to its slot
- "put_back": move a used tool from the reclaim tray back to the instrument tray
- "release": open the gripper now
- "home": move the arm to its home pose
- "abort": stop the current robot motion
- "register": start the operation, freeze the tray as its inventory
- "count": the instrument count at the end of the operation
- "unknown": none of the above, or you cannot tell

Instrument classes:
{tools_desc}
tool_class must be exactly one class name from the list, or null when no
instrument is named. The transcript comes from speech recognition and may be
garbled — resolve mispronunciations to the closest instrument. If the surgeon
could mean several instruments, use the most likely one; pick "unknown" only
when you truly cannot tell."""

    def _get_client(self):
        if self._client is None:
            try:
                from openai import OpenAI
                self._client = OpenAI(api_key=self._api_key,
                                      timeout=self._timeout_sec)
            except ImportError:
                self._log('error', 'openai not installed: pip install openai')
                return None
            except Exception as e:
                self._log('error', f'OpenAI client init failed: {e}')
                return None
        return self._client

    def classify(self, text: str) -> Optional[Tuple[str, Optional[str]]]:
        """One utterance -> (action, tool_class or None). None = backend failed
        (no network, bad output); the caller falls back to UNKNOWN."""
        client = self._get_client()
        if client is None:
            return None
        try:
            resp = client.chat.completions.create(
                model=self._model_name,
                messages=[
                    {"role": "system", "content": self._system_prompt},
                    {"role": "user", "content": text},
                ],
                response_format={"type": "json_object"},
                # Includes GPT-5's hidden reasoning tokens as well as the
                # small JSON object returned to the router.
                max_completion_tokens=256,
            )
        except Exception as e:
            self._log('warn', f'LLM intent fallback failed: {e}')
            return None

        try:
            payload = json.loads(resp.choices[0].message.content or '{}')
        except (json.JSONDecodeError, IndexError) as e:
            self._log('warn', f'LLM intent fallback: bad JSON ({e})')
            return None

        # Strict validation: the model proposes, the catalog disposes.
        action = str(payload.get('action', 'unknown')).strip().lower()
        if action not in _ACTIONS:
            self._log('warn', f'LLM intent fallback: bad action "{action}"')
            action = 'unknown'
        tool_class = payload.get('tool_class')
        if tool_class is not None:
            tool_class = str(tool_class).strip()
            if tool_class not in self._catalog:
                self._log('warn',
                          f'LLM intent fallback: unknown class "{tool_class}"')
                tool_class = None
        return action, tool_class
