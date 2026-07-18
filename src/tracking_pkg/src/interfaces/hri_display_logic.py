#!/usr/bin/env python3
"""Pure display logic for the HRI traffic light — no ROS, no cv2, unit-testable.

Given the latest robot signals, decide the colour, headline word and animation.
The node in hri_display_node.py is a thin ROS+OpenCV shell around this.
"""

# State groups from /system_state_update ("state:tool_id:tool_class").
_MOVING = {'PICKING', 'TRANSPORTING', 'RETURNING', 'RELEASING', 'HANDOVER'}
_TAKE = {'PRESENTING'}

# /system_response substrings that warrant a transient red alert. Kept tight:
# plain "no tool on tray" / "which one?" are info, not alarms.
_ALERT_SUBSTRINGS = (
    'cannot', 'keeps dropping', 'dropped', 'missed', 'failed', 'error',
    'unavailable', 'timed out', 'out of reach',
)


def pretty_tool(tool_class):
    return tool_class.replace('_', ' ').title() if tool_class else ''


def wrap_lines(text, max_chars, max_lines=3):
    """Greedy word-wrap into <= max_lines lines of <= max_chars each. The last
    line is ellipsised if the text still overflows, so a long response reads as
    several lines instead of one cut-off line."""
    text = (text or '').strip()
    if not text:
        return []
    words, lines, cur = text.split(), [], ''
    for w in words:
        cand = w if not cur else cur + ' ' + w
        if len(cand) <= max_chars or not cur:
            cur = cand
        else:
            lines.append(cur)
            cur = w
        if len(lines) == max_lines:
            break
    if cur and len(lines) < max_lines:
        lines.append(cur)
    # Anything left over -> ellipsis on the last line.
    packed = ' '.join(lines)
    if len(packed) < len(text):
        last = lines[-1]
        if len(last) >= max_chars:
            last = last[:max_chars - 1].rstrip()
        lines[-1] = last + '…'
    return lines


def is_alert_response(text):
    """Does this /system_response line warrant a transient red alert?"""
    low = (text or '').lower()
    return any(s in low for s in _ALERT_SUBSTRINGS)


def resolve_display(alert_active, alert_text, system_state, active_tool_class,
                    last_response, handover_waiting, asr_listening, pulse_take):
    """Priority resolution -> (color_key, headline, detail, anim).

    color_key ∈ {red, amber, green}; anim ∈ {steady, blink, pulse}.
    Priority: transient alert > arm moving > present/take > your-turn
    (gesture/listening) > idle.
    """
    if alert_active:
        return ('red', 'ALERT', alert_text, 'blink')
    if system_state in _MOVING:
        tool = pretty_tool(active_tool_class)
        if system_state == 'RETURNING':
            detail = 'Returning'
        elif tool:
            detail = f'Fetching {tool}'
        else:
            detail = last_response or 'Moving'
        return ('red', 'BUSY', detail, 'steady')
    if system_state in _TAKE:
        tool = pretty_tool(active_tool_class)
        detail = f'Take {tool}' if tool else 'Take the tool'
        return ('green', 'TAKE', detail, 'pulse' if pulse_take else 'steady')
    if system_state == 'AWAIT_GESTURE' or handover_waiting:
        return ('amber', 'GESTURE', 'Make your gesture', 'steady')
    if asr_listening:
        return ('amber', 'SPEAK', 'Speak now', 'steady')
    return ('green', 'READY', last_response or 'Ready', 'steady')


class Debouncer:
    """Suppress sub-`debounce_sec` flicker between resolved states. Safety wins:
    a red/alert state commits immediately; only calmer states wait."""

    def __init__(self, debounce_sec):
        self.debounce_sec = float(debounce_sec)
        self._shown = None
        self._pending = None
        self._pending_since = 0.0

    def commit(self, resolved, now):
        if self._shown is None:
            self._shown = self._pending = resolved
            self._pending_since = now
            return resolved
        if resolved == self._shown:
            self._pending = resolved
            self._pending_since = now
            return self._shown
        if resolved != self._pending:
            self._pending = resolved
            self._pending_since = now
        # Immediate for safety-relevant red / alerts; debounce the rest.
        if resolved[0] == 'red' or resolved[3] == 'blink' \
                or (now - self._pending_since) >= self.debounce_sec:
            self._shown = resolved
        return self._shown
