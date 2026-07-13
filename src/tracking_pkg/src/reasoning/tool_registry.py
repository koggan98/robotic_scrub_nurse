#!/usr/bin/env python3
"""The instrument inventory of one operation.

A scrub nurse counts the instruments before the operation and again at the end.
Anything that does not come back is, until proven otherwise, still inside the
patient. This module is that count.

The registry holds one SLOT per physical instrument. The slot — not the
perception track — is the identity:

  * `ToolTracker` ids (tool_3, reclaim_1) are NOT durable. Tracks are evicted
    after 3 s (instrument) / 10 s (reclaim), and a tool that was briefly occluded
    comes back with a BRAND NEW id. Two same-class tools closer than the tracker's
    5 cm threshold can even swap ids between frames.
  * So a slot is anchored to its HOME POSITION, and track ids are re-bound to
    slots on every observation, never stored.

The slot stores the tool's HOME POSE — handle centre + long axis — not a grasp
pose. The grasp point is `handle_center + d * functional_end_dir`, and `d` differs
per tray (the sliding-into-the-opening strategy depends on that tray's hole). So
placing a tool back at its recorded *grasp* pose would leave it displaced along
its own axis by `d_reclaim - d_home`. Only the tool pose is invariant.

No ROS types in here — pure geometry and bookkeeping, so the whole state machine
is testable without a robot.
"""

import json
import math
import os
import tempfile
import time

# ── States. Only what is actually observable. ────────────────────────
# "lost" is deliberately NOT a state: while the surgeon holds a tool it is simply
# IN_USE, which is normal. "Lost" is a VERDICT, and it is passed at count time by
# a human looking at the report.
AT_HOME = 'AT_HOME'        # on the instrument tray, at its slot — ready for use
IN_GRIPPER = 'IN_GRIPPER'  # the robot holds it
IN_USE = 'IN_USE'          # handed over, nowhere visible — with the surgeon, or in the patient
ON_RECLAIM = 'ON_RECLAIM'  # on the reclaim tray (parked for re-use, OR awaiting return)
UNKNOWN = 'UNKNOWN'        # registered, but never seen since and no event explains it

INSTRUMENT_TRAY = 'instrument_tray'
RECLAIM_TRAY = 'reclaim_tray'


class Slot:
    """One physical instrument."""

    __slots__ = ('slot_id', 'tool_class', 'state', 'home_xy', 'home_dir',
                 'home_plane_z', 'last_seen_sec', 'last_state_change_sec',
                 'track_id')

    def __init__(self, slot_id, tool_class, home_xy, home_dir, home_plane_z,
                 now=0.0):
        self.slot_id = slot_id
        self.tool_class = tool_class
        self.state = AT_HOME
        self.home_xy = tuple(float(v) for v in home_xy)      # handle centre (world XY)
        self.home_dir = tuple(float(v) for v in home_dir)    # functional_end_dir (world XY)
        self.home_plane_z = float(home_plane_z)
        self.last_seen_sec = float(now)
        self.last_state_change_sec = float(now)
        self.track_id = ''       # re-bound every observation, NEVER persisted as identity

    def set_state(self, state, now):
        if state != self.state:
            self.state = state
            self.last_state_change_sec = float(now)

    def to_dict(self):
        return {
            'slot_id': self.slot_id,
            'tool_class': self.tool_class,
            'state': self.state,
            'home_xy': list(self.home_xy),
            'home_dir': list(self.home_dir),
            'home_plane_z': self.home_plane_z,
            'last_seen_sec': self.last_seen_sec,
            'last_state_change_sec': self.last_state_change_sec,
        }

    @classmethod
    def from_dict(cls, d):
        s = cls(d['slot_id'], d['tool_class'], d['home_xy'], d['home_dir'],
                d['home_plane_z'])
        s.state = d.get('state', UNKNOWN)
        s.last_seen_sec = d.get('last_seen_sec', 0.0)
        s.last_state_change_sec = d.get('last_state_change_sec', 0.0)
        return s


class ToolRegistry:
    def __init__(self, min_same_class_gap_m=0.08, home_match_radius_m=0.12):
        # Two tools of the SAME class closer than this cannot be told apart
        # reliably: the ToolTracker's greedy matcher (5 cm threshold, YOLO
        # detection order) can swap their ids between frames, and the slot binding
        # would silently jump. Registration refuses rather than be quietly wrong.
        self.min_same_class_gap_m = float(min_same_class_gap_m)
        # How far from its home a tool may be detected and still count as "at home".
        self.home_match_radius_m = float(home_match_radius_m)
        self.slots = {}          # slot_id -> Slot
        self.registered_at = 0.0

    # ── Registration ────────────────────────────────────────────────

    def register(self, observations, now=None):
        """Freeze whatever is on the instrument tray as this operation's inventory.

        `observations`: list of dicts {tool_class, handle_xy, end_dir, plane_z}.

        There is NO expected list to check against — four tools are as valid an
        inventory as eleven. What we DO check is that same-class duplicates are far
        enough apart to be tracked apart (see min_same_class_gap_m).

        Returns (ok, found: {class: count}, message).
        """
        now = time.time() if now is None else now

        by_class = {}
        for o in observations:
            by_class.setdefault(o['tool_class'], []).append(o)

        # Reject unreliable duplicate layouts — actionably, not silently.
        for cls, obs in by_class.items():
            for i in range(len(obs)):
                for j in range(i + 1, len(obs)):
                    d = _dist(obs[i]['handle_xy'], obs[j]['handle_xy'])
                    if d < self.min_same_class_gap_m:
                        return (False, {}, (
                            f'two {cls} are only {d * 100:.0f} cm apart — the tracker '
                            f'cannot tell them apart below '
                            f'{self.min_same_class_gap_m * 100:.0f} cm. '
                            f'Move them further apart and register again.'))

        self.slots.clear()
        found = {}
        for cls, obs in sorted(by_class.items()):
            for n, o in enumerate(obs, start=1):
                slot_id = f'{cls}_{n}'
                self.slots[slot_id] = Slot(
                    slot_id, cls, o['handle_xy'], o['end_dir'], o['plane_z'], now)
            found[cls] = len(obs)

        self.registered_at = now
        total = sum(found.values())
        return (True, found, f'registered {total} instruments')

    @property
    def is_registered(self):
        return bool(self.slots)

    # ── Observation: detections -> slots ────────────────────────────

    def observe(self, location, observations, now=None):
        """Bind this frame's detections to slots and update their state.

        Track ids are re-bound here on every call and never trusted across frames
        (see the module docstring). Absence of a detection is NOT evidence — a tool
        that is not seen keeps its state. Only a positive observation, or an
        explicit ToolEvent, moves a slot.
        """
        now = time.time() if now is None else now
        if not self.is_registered:
            return

        if location == INSTRUMENT_TRAY:
            self._observe_instrument(observations, now)
        elif location == RECLAIM_TRAY:
            self._observe_reclaim(observations, now)

    def _observe_instrument(self, observations, now):
        used = set()
        for o in observations:
            slot = self._nearest_free_home(o['tool_class'], o['handle_xy'], used)
            if slot is None:
                continue          # e.g. a tool of a class that was never registered
            used.add(slot.slot_id)
            slot.track_id = o.get('track_id', '')
            slot.last_seen_sec = now
            # A tool physically lying on the instrument tray IS at home, whatever
            # we believed a moment ago. Perception beats bookkeeping.
            slot.set_state(AT_HOME, now)

    def _observe_reclaim(self, observations, now):
        used = set()
        for o in observations:
            cls = o['tool_class']
            # Which instance is it? Any of that class that is NOT at home. They are
            # physically identical, so it does not matter which — but prefer the one
            # that has been out the longest, so repeated returns drain FIFO.
            cands = [s for s in self.slots.values()
                     if s.tool_class == cls
                     and s.slot_id not in used
                     and s.state != AT_HOME]
            if not cands:
                # It is on the reclaim tray but we thought every instance was at
                # home. Perception wins: take the one seen longest ago.
                cands = [s for s in self.slots.values()
                         if s.tool_class == cls and s.slot_id not in used]
            if not cands:
                continue
            slot = min(cands, key=lambda s: s.last_state_change_sec)
            used.add(slot.slot_id)
            slot.track_id = o.get('track_id', '')
            slot.last_seen_sec = now
            slot.set_state(ON_RECLAIM, now)

    def _nearest_free_home(self, tool_class, xy, used):
        best, best_d = None, self.home_match_radius_m
        for s in self.slots.values():
            if s.tool_class != tool_class or s.slot_id in used:
                continue
            d = _dist(s.home_xy, xy)
            if d < best_d:
                best, best_d = s, d
        return best

    # ── Events from the executor ────────────────────────────────────

    def on_event(self, event, track_id, tool_class, from_location='', now=None):
        """Apply a ToolEvent. These are the transitions perception CANNOT see.

        Returns the affected Slot, or None.
        """
        now = time.time() if now is None else now
        if not self.is_registered:
            return None

        slot = self._slot_for_track(track_id, tool_class, from_location)
        if slot is None:
            return None

        if event == 'PICKED':
            slot.set_state(IN_GRIPPER, now)
        elif event == 'HANDED_OVER':
            # The surgeon has physically pulled it out of the jaws (/gripper_done).
            # From here it is invisible to every camera — and that is normal.
            slot.set_state(IN_USE, now)
        elif event == 'PLACED_HOME':
            slot.set_state(AT_HOME, now)
        elif event in ('RELEASED', 'DROPPED'):
            # Out of the gripper, but we do not know onto what. Perception will
            # correct this within a frame or two if it landed on a tray.
            slot.set_state(UNKNOWN, now)
        return slot

    def _slot_for_track(self, track_id, tool_class, from_location):
        # A live track id is only meaningful within the current frame, but the
        # executor read it from the same snapshot, so it is still fresh here.
        if track_id:
            for s in self.slots.values():
                if s.track_id == track_id:
                    return s
        # Fall back to the class. For a pick, the tool in the gripper is the one of
        # that class that most recently left its tray.
        cands = [s for s in self.slots.values() if s.tool_class == tool_class]
        if not cands:
            return None
        if from_location == RECLAIM_TRAY:
            on_reclaim = [s for s in cands if s.state == ON_RECLAIM]
            if on_reclaim:
                return max(on_reclaim, key=lambda s: s.last_seen_sec)
        in_gripper = [s for s in cands if s.state == IN_GRIPPER]
        if in_gripper:
            return in_gripper[0]
        return max(cands, key=lambda s: s.last_seen_sec)

    # ── Queries ─────────────────────────────────────────────────────

    def home_of(self, tool_class, track_id=''):
        """The slot a tool of this class belongs to — i.e. where to put it back.

        Prefers the instance we believe is currently out, so returning a tool while
        its twin sits at home does not target the occupied slot.
        """
        if track_id:
            for s in self.slots.values():
                if s.track_id == track_id:
                    return s
        cands = [s for s in self.slots.values() if s.tool_class == tool_class]
        if not cands:
            return None
        away = [s for s in cands if s.state != AT_HOME]
        pool = away or cands
        return min(pool, key=lambda s: s.last_state_change_sec)

    def count(self):
        """The instrument count: the registered inventory against what we see now."""
        buckets = {AT_HOME: [], IN_GRIPPER: [], IN_USE: [], ON_RECLAIM: [],
                   UNKNOWN: []}
        for s in self.slots.values():
            buckets[s.state].append(s.slot_id)

        expected = len(self.slots)
        at_home = len(buckets[AT_HOME])
        unaccounted = (buckets[IN_USE] + buckets[UNKNOWN])
        return {
            'registered': self.is_registered,
            'expected': expected,
            'at_home': at_home,
            'all_accounted_for': expected > 0 and at_home == expected,
            'in_gripper': sorted(buckets[IN_GRIPPER]),
            'on_reclaim': sorted(buckets[ON_RECLAIM]),
            # The critical ones: nowhere visible.
            'in_use': sorted(buckets[IN_USE]),
            'unknown': sorted(buckets[UNKNOWN]),
            'unaccounted_for': sorted(unaccounted),
        }

    def snapshot(self):
        return {
            'registered': self.is_registered,
            'registered_at_sec': self.registered_at,
            'tools': [
                {
                    'slot_id': s.slot_id,
                    'class': s.tool_class,
                    'state': s.state,
                    'track_id': s.track_id,
                    'home_xy': list(s.home_xy),
                }
                for s in sorted(self.slots.values(), key=lambda x: x.slot_id)
            ],
        }

    # ── Persistence ─────────────────────────────────────────────────
    # The share dir is read-only after install, so state lives under the user's
    # home (same idea as scrub_nurse_logger). Without this a crash of
    # world_model_node mid-operation means the whole inventory is gone — and with
    # it the count that the feature exists for.

    def save(self, path):
        data = {
            'registered_at': self.registered_at,
            'slots': [s.to_dict() for s in self.slots.values()],
        }
        os.makedirs(os.path.dirname(path), exist_ok=True)
        # Atomic: a half-written registry is worse than none.
        fd, tmp = tempfile.mkstemp(dir=os.path.dirname(path), suffix='.tmp')
        try:
            with os.fdopen(fd, 'w') as f:
                json.dump(data, f, indent=2)
            os.replace(tmp, path)
        except Exception:
            if os.path.exists(tmp):
                os.unlink(tmp)
            raise

    def load(self, path):
        with open(path, 'r') as f:
            data = json.load(f) or {}
        self.slots = {
            d['slot_id']: Slot.from_dict(d) for d in data.get('slots', [])
        }
        self.registered_at = data.get('registered_at', 0.0)
        return len(self.slots)


def _dist(a, b):
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))
