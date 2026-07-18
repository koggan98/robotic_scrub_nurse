#!/usr/bin/env python3
"""Deterministic intent parser: transcript -> Intent{action, tool_class}.

This is the offline half of the hybrid NLU. It resolves the surgeon's utterance
with a verb lexicon (command_lexicon.yaml) and a fuzzy synonym match against the
tool catalog (tool_catalog.yaml). No network, no LLM, no ROS — pure functions
over two YAML files, so the whole thing is unit-testable on any machine.

Why fuzzy and not exact: the transcript comes from Whisper, which reliably
mangles instrument names ("sweezer", "needle holler"). Synonym resolution used
to be an LLM prompt rule that the model was free to ignore ("no tweezers, only
forceps"); here it is a lookup, so a synonym in the catalog is a guarantee.

Decision ladder for the tool name (scores are difflib ratios in [0, 1]):
  1. best >= threshold and clearly ahead of the runner-up  -> confident match
  2. best >= floor with near-ties (or not clearly ahead)   -> candidates, the
     router asks "Which one?" — this is what a bare "scissors" or "tweezers"
     hits, since the catalog only knows long/short and big/mid/small variants
  3. best < floor and no action verb                       -> UNKNOWN, the
     router may hand the raw text to the LLM fallback

ABORT is matched before everything else: "stop" must win over any other word
in the utterance, because it stops a moving arm.
"""

import re
from dataclasses import dataclass, field
from difflib import SequenceMatcher
from enum import Enum
from typing import Dict, List, Optional


class Action(Enum):
    PICK = 'pick'
    RETURN = 'return'          # bring the held (wrong) tool back to its slot
    PUT_BACK = 'put_back'      # reclaim tray -> instrument tray
    RELEASE = 'release'
    HOME = 'home'
    ABORT = 'abort'
    REGISTER = 'register'
    COUNT = 'count'
    UNKNOWN = 'unknown'


# Match priority. ABORT first — safety. RETURN ("wrong") before BACK ("back"),
# so "wrong one, put it back" reads as a return of the held tool, not as
# reclaim-tray housekeeping. The `back` lexicon group is not an Action of its
# own: with a named tool it becomes PUT_BACK, bare it becomes RETURN (and the
# router re-interprets RETURN as PUT_BACK when the gripper is empty).
_VERB_PRIORITY = ['abort', 'return', 'back', 'count', 'register',
                  'release', 'home']

_VERB_TO_ACTION = {
    'abort': Action.ABORT,
    'return': Action.RETURN,
    'count': Action.COUNT,
    'register': Action.REGISTER,
    'release': Action.RELEASE,
    'home': Action.HOME,
}


@dataclass
class Intent:
    action: Action
    tool_class: Optional[str] = None
    candidates: List[str] = field(default_factory=list)  # near-tie classes
    raw: str = ''
    score: float = 0.0
    source: str = 'deterministic'    # 'deterministic' | 'llm'
    matched_phrase: str = ''         # which lexicon phrase / synonym fired


def normalize(text: str) -> str:
    """Lowercase, strip punctuation (apostrophes survive: "we're"), collapse
    whitespace. Same treatment for transcript and lexicon so they meet in the
    middle."""
    text = text.lower()
    text = re.sub(r"[^\w\s']", ' ', text, flags=re.UNICODE)
    return ' '.join(text.split())


class IntentParser:
    def __init__(self, catalog: Dict, lexicon: Dict,
                 fuzzy_threshold: float = 0.8,
                 fuzzy_floor: float = 0.6,
                 near_tie_delta: float = 0.1,
                 verb_fuzzy_threshold: float = 0.82):
        self.fuzzy_threshold = float(fuzzy_threshold)
        self.fuzzy_floor = float(fuzzy_floor)
        self.near_tie_delta = float(near_tie_delta)
        # Verb phrases get a fuzzy pass too — Whisper turns "end surgery" into
        # "and surgery" and "finish" into "Finnish". Higher bar than the tools
        # (0.82 keeps "please" from becoming "release"), and phrases shorter
        # than 5 characters stay exact-only so "shop" can never abort the arm.
        self.verb_fuzzy_threshold = float(verb_fuzzy_threshold)

        # class -> normalized synonym strings. The class name itself and the
        # display name count as synonyms too ("needle_holder" is spoken
        # "needle holder").
        self._synonyms: Dict[str, List[str]] = {}
        for cls, info in (catalog or {}).items():
            syns = {normalize(cls.replace('_', ' '))}
            if info.get('display_name'):
                syns.add(normalize(info['display_name']))
            for s in info.get('synonyms', []) or []:
                n = normalize(s)
                if n:
                    syns.add(n)
            self._synonyms[cls] = sorted(syns)

        # verb group -> [(normalized phrase, compiled word-boundary regex)]
        self._verbs: Dict[str, List] = {}
        for group, phrases in ((lexicon or {}).get('actions', {}) or {}).items():
            entries = []
            for p in phrases or []:
                n = normalize(str(p))
                if n:
                    entries.append(
                        (n, re.compile(r'\b' + re.escape(n) + r'\b',
                                       re.UNICODE)))
            self._verbs[group] = entries

    # ── Public API ──────────────────────────────────────────────────

    def parse(self, text: str) -> Intent:
        raw = text
        norm = normalize(text)
        if not norm:
            return Intent(Action.UNKNOWN, raw=raw)

        verb_group, verb_phrase = self._match_verb(norm)
        tool_cls, score, cands, syn = self._match_tool(norm)

        # ABORT beats everything, including a named tool in the same breath.
        if verb_group == 'abort':
            return Intent(Action.ABORT, raw=raw, matched_phrase=verb_phrase)

        if verb_group == 'return':
            return Intent(Action.RETURN, raw=raw, matched_phrase=verb_phrase)

        if verb_group == 'back':
            if tool_cls:
                return Intent(Action.PUT_BACK, tool_class=tool_cls, raw=raw,
                              score=score, matched_phrase=syn)
            if cands:
                return Intent(Action.PUT_BACK, candidates=cands, raw=raw,
                              score=score, matched_phrase=verb_phrase)
            # Bare "put it back": about whatever the robot is holding.
            return Intent(Action.RETURN, raw=raw, matched_phrase=verb_phrase)

        if verb_group in _VERB_TO_ACTION and verb_group != 'return':
            return Intent(_VERB_TO_ACTION[verb_group], raw=raw,
                          matched_phrase=verb_phrase)

        # No verb: a named instrument is a pick request.
        if tool_cls:
            return Intent(Action.PICK, tool_class=tool_cls, raw=raw,
                          score=score, matched_phrase=syn)
        if cands:
            return Intent(Action.PICK, candidates=cands, raw=raw, score=score)

        return Intent(Action.UNKNOWN, raw=raw, score=score)

    def resolve_tool_choice(self, text: str,
                            allowed_classes: List[str]) -> Optional[str]:
        """Resolve a short answer against an explicitly offered class list.

        This is intentionally narrower than ``parse``: after the router asked
        "Large Forceps or Hammer or Small Forceps?", answers such as
        "forceps small", "small", or "hammer" are choices, not new commands.
        Generic family words (for example just "forceps") remain ambiguous.
        """
        norm = normalize(text)
        allowed = [c for c in allowed_classes if c in self._synonyms]
        if not norm or not allowed:
            return None

        tokens = norm.split()
        token_bag = sorted(tokens)
        exact = []
        for cls in allowed:
            if any(norm == syn or token_bag == sorted(syn.split())
                   for syn in self._synonyms[cls]):
                exact.append(cls)
        if len(exact) == 1:
            return exact[0]

        # A unique descriptive word is enough inside a closed choice. Remove
        # conversational fillers and generic instrument-family nouns first, so
        # "small one" selects Small Forceps while bare "forceps" selects none.
        generic = {
            'a', 'an', 'the', 'one', 'please', 'tool', 'instrument',
            'forceps', 'tweezers', 'pinzette', 'scissors', 'schere',
        }
        informative = set(tokens) - generic
        if informative:
            matches = []
            for cls in allowed:
                vocabulary = {
                    token
                    for syn in self._synonyms[cls]
                    for token in syn.split()
                }
                if informative <= vocabulary:
                    matches.append(cls)
            if len(matches) == 1:
                return matches[0]

        # Retain the existing ASR-damage tolerance, but restrict it to choices
        # the robot actually offered. Never accept another ambiguous result.
        tool_cls, _score, _cands, _syn = self._match_tool(
            norm, allowed_classes=allowed)
        return tool_cls

    # ── Verb matching ───────────────────────────────────────────────

    def _match_verb(self, norm: str):
        # Exact pass first, in priority order — cheap and unambiguous.
        for group in _VERB_PRIORITY:
            for phrase, rx in self._verbs.get(group, []):
                if rx.search(norm):
                    return group, phrase
        # Fuzzy pass: ASR-damaged verbs ("Finnish surgery", "and surgery").
        # Only phrases of >= 5 characters take part; ties go to the higher-
        # priority group because iteration is in priority order and only a
        # strictly better score displaces the incumbent.
        tokens = norm.split()
        best_score, best_group, best_phrase = 0.0, None, ''
        for group in _VERB_PRIORITY:
            for phrase, _rx in self._verbs.get(group, []):
                if len(phrase) < 5:
                    continue
                n = len(phrase.split())
                for w in range(max(1, n - 1), n + 2):
                    for i in range(max(1, len(tokens) - w + 1)):
                        window = ' '.join(tokens[i:i + w])
                        s = SequenceMatcher(None, window, phrase).ratio()
                        if s > best_score:
                            best_score, best_group, best_phrase = s, group, phrase
        if best_score >= self.verb_fuzzy_threshold:
            return best_group, best_phrase
        return None, ''

    # ── Tool matching ───────────────────────────────────────────────

    def _match_tool(self, norm: str, allowed_classes=None):
        """Best fuzzy match of the transcript against every synonym.

        Returns (confident_class | None, best_score, candidate_classes,
        matched_synonym). Sliding token windows sized to each synonym (±1, so
        Whisper merging "needle holder" into "needleholder" still scores).
        """
        tokens = norm.split()
        best_per_class = {}   # cls -> (score, synonym)
        for cls, syns in self._synonyms.items():
            if allowed_classes is not None and cls not in allowed_classes:
                continue
            cls_best, cls_syn = 0.0, ''
            for syn in syns:
                n = len(syn.split())
                for w in range(max(1, n - 1), n + 2):
                    for i in range(max(1, len(tokens) - w + 1)):
                        window = ' '.join(tokens[i:i + w])
                        # Whisper occasionally reverses adjective+noun into
                        # noun+adjective ("small forceps" -> "forceps small").
                        # The same complete tokens are still an exact name.
                        if w == n and sorted(window.split()) == sorted(syn.split()):
                            s = 1.0
                        else:
                            s = SequenceMatcher(None, window, syn).ratio()
                        if s > cls_best:
                            cls_best, cls_syn = s, syn
            if cls_best > 0.0:
                best_per_class[cls] = (cls_best, cls_syn)

        if not best_per_class:
            return None, 0.0, [], ''

        ranked = sorted(best_per_class.items(),
                        key=lambda kv: kv[1][0], reverse=True)
        top_cls, (top_score, top_syn) = ranked[0]
        runners = [cls for cls, (s, _) in ranked[1:]
                   if s >= max(self.fuzzy_floor, top_score - self.near_tie_delta)]

        if top_score >= self.fuzzy_threshold and not runners:
            return top_cls, top_score, [], top_syn
        if top_score >= self.fuzzy_floor:
            # Near-ties, or a single sub-threshold guess: let the router ask.
            return None, top_score, [top_cls] + runners, top_syn
        return None, top_score, [], ''
