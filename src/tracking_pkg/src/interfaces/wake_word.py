#!/usr/bin/env python3
"""Wake-word gate for the ASR pipeline. Pure — no ROS, unit-testable.

The microphone hears everything said near the table; only utterances addressed
to the robot ("robot, needle holder") may become commands. The gate also kills
Whisper's noise hallucinations ("Thank you.") for free, since those never start
with the wake word.

Matching is fuzzy because Whisper reliably re-spells short names: "robi" comes
back as "Robbie" or "Roby", "robot" occasionally as "robots". The wake word
must be the FIRST word, or preceded only by known fillers ("hey robot",
"ok robot") — an article in front ("the robot is over there") is conversation
ABOUT the robot, not a command to it, and stays blocked. Everything up to and
including the wake token is stripped; the command keeps its original spelling
for the parser downstream.
"""

import re
from difflib import SequenceMatcher
from typing import List, Optional

# Words allowed BEFORE the wake word. Anything else in front means the
# utterance only mentions the robot instead of addressing it.
_FILLERS = {'hey', 'ok', 'okay', 'so', 'hi', 'hello', 'oh', 'ey', 'and',
            'now', 'ähm', 'uh', 'um'}


def _norm_token(token: str) -> str:
    """Lowercase and strip punctuation ("Robot," -> "robot")."""
    return re.sub(r"[^\w']", '', token.lower(), flags=re.UNICODE)


def strip_wake_word(text: str, wake_words: List[str],
                    fuzzy_threshold: float = 0.75) -> Optional[str]:
    """The command after the wake word, or None when no wake word was heard.

    Returns '' when the utterance was ONLY the wake word — the caller decides
    whether that means anything.
    """
    if not wake_words:
        return text  # gate disabled
    tokens = text.split()
    wake = [w.strip().lower() for w in wake_words if w.strip()]
    for i, tok in enumerate(tokens):
        norm = _norm_token(tok)
        if not norm:
            continue
        for w in wake:
            if norm == w or SequenceMatcher(None, norm, w).ratio() >= fuzzy_threshold:
                return ' '.join(tokens[i + 1:])
        if norm not in _FILLERS:
            return None  # a real word before any wake word: not addressed to us
    return None


def plan_wake_segment(phase, text, wake_words, fuzzy_threshold=0.75,
                      two_stage=True):
    """Decide what a transcribed segment means in the two-stage wake FSM.

    Pure — no ROS, no side effects — so the branching is unit-testable. Returns
    one of:
      ('command', <cmd>)  publish <cmd> as /user_speech; stay in wait_wake
      ('arm',)            wake word only -> enter the command phase (armed)
      ('disarm', None)    command window ended with nothing usable -> idle
      ('ignore', None)    no wake word / nothing to do

    `text` is the transcript or None (empty/short/timed-out segment).
    """
    # Wake gate disabled: every segment is a command.
    if not wake_words:
        return ('command', text) if text else ('ignore', None)

    if phase == 'command':
        if not text:
            return ('disarm', None)
        # They may have re-said the wake word; keep the command part if so.
        stripped = strip_wake_word(text, wake_words, fuzzy_threshold)
        return ('command', stripped if stripped else text)

    # phase == 'wait_wake'
    if not text:
        return ('ignore', None)
    command = strip_wake_word(text, wake_words, fuzzy_threshold)
    if command is None:
        return ('ignore', None)          # no wake word
    if command:
        return ('command', command)      # "robot needle holder" in one breath
    # Wake word only.
    return ('arm',) if two_stage else ('ignore', None)
