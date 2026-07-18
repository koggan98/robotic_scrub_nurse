#!/usr/bin/env python3
"""Offline unit tests for the deterministic intent parser.

Runs against the REAL tool_catalog.yaml and command_lexicon.yaml, so a config
edit that breaks command understanding fails here — before it reaches a robot.
No ROS required:

    python3 -m pytest src/tracking_pkg/test/test_intent_parser.py -q
"""

import os
import sys

import pytest
import yaml

_PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_PKG, 'src', 'llm'))

from intent_parser import Action, IntentParser, normalize  # noqa: E402


@pytest.fixture(scope='module')
def parser():
    with open(os.path.join(_PKG, 'config', 'tool_catalog.yaml')) as f:
        catalog = yaml.safe_load(f)['catalog']
    with open(os.path.join(_PKG, 'config', 'command_lexicon.yaml')) as f:
        lexicon = yaml.safe_load(f)
    return IntentParser(catalog, lexicon)


# ── Confident tool picks (no verb -> PICK) ──────────────────────────

@pytest.mark.parametrize('text,expected_class', [
    ('needle holder', 'needle_holder'),
    ('give me the needle holder please', 'needle_holder'),
    ('needle holler', 'needle_holder'),        # ASR damage
    ('needleholder', 'needle_holder'),         # Whisper merged the words
    ('nadelhalter', 'needle_holder'),          # German
    ('porte-aiguille', 'needle_holder'),       # French
    ('lange schere', 'scissors_long'),
    ('mayo', 'scissors_long'),
    ('metzenbaum', 'scissors_short'),
    ('small forceps', 'forceps_short'),
    ('forceps small', 'forceps_short'),       # Whisper reversed word order
    ('small tweezers', 'forceps_short'),
    ('large forceps', 'forceps_big'),
    ('hammer', 'hammer'),
    ('the awl', 'awl'),
    ('retractor', 'retractor'),
    ('langenbeck', 'retractor'),
])
def test_confident_pick(parser, text, expected_class):
    intent = parser.parse(text)
    assert intent.action == Action.PICK
    assert intent.tool_class == expected_class
    assert intent.source == 'deterministic'


# ── Ambiguity: ask, never guess ─────────────────────────────────────

@pytest.mark.parametrize('text,expected_prefix', [
    ('scissors', 'scissors'),   # long vs short — the catalog has no bare entry
    ('forceps', 'forceps'),     # big vs mid vs small
    ('tweezers', 'forceps'),    # synonym of two forceps sizes
    ('pinzette', 'forceps'),
])
def test_ambiguous_tool_yields_candidates(parser, text, expected_prefix):
    intent = parser.parse(text)
    assert intent.action == Action.PICK
    assert intent.tool_class is None
    assert len(intent.candidates) >= 2
    assert all(c.startswith(expected_prefix) for c in intent.candidates)


def test_garbled_synonym_never_resolves_to_wrong_class(parser):
    # "sweezer" is damaged "tweezers": must never confidently become a
    # non-forceps class. Candidates or UNKNOWN are both acceptable.
    intent = parser.parse('sweezer')
    if intent.tool_class is not None:
        assert intent.tool_class.startswith('forceps')
    elif intent.candidates:
        assert all(c.startswith('forceps') for c in intent.candidates)
    else:
        assert intent.action == Action.UNKNOWN


@pytest.mark.parametrize('text,expected', [
    ('forceps small', 'forceps_short'),
    ('small', 'forceps_short'),
    ('the large one', 'forceps_big'),
    ('hammer', 'hammer'),
    ('forceps', None),
    ('medium', None),  # not among the offered/available choices
])
def test_resolve_closed_pick_choice(parser, text, expected):
    offered = ['forceps_big', 'hammer', 'forceps_short']
    assert parser.resolve_tool_choice(text, offered) == expected


# ── Verbs ───────────────────────────────────────────────────────────

@pytest.mark.parametrize('text,action', [
    ('stop', Action.ABORT),
    ('abort abort', Action.ABORT),
    ('wrong tool', Action.RETURN),
    ('not that one', Action.RETURN),
    ('put it back', Action.RETURN),            # bare back = the held tool
    ('count the instruments', Action.COUNT),
    ("we're done", Action.COUNT),
    ('end of operation', Action.COUNT),
    ('end surgery', Action.COUNT),
    ('finish surgery', Action.COUNT),
    ('setup ready', Action.REGISTER),
    ('start the operation', Action.REGISTER),
    ('start surgery', Action.REGISTER),
    ('release', Action.RELEASE),
    ('open the gripper', Action.RELEASE),
    ('let go', Action.RELEASE),
    ('go home', Action.HOME),
])
def test_verbs(parser, text, action):
    assert parser.parse(text).action == action


# ── ASR-damaged verbs: the fuzzy pass must catch these ──────────────

@pytest.mark.parametrize('text,action', [
    ('and surgery.', Action.COUNT),        # Whisper heard "end surgery"
    ('Finnish surgery', Action.COUNT),     # Whisper heard "finish surgery"
    ('Stark surgery', Action.REGISTER),    # Whisper heard "start surgery"
])
def test_fuzzy_verbs(parser, text, action):
    assert parser.parse(text).action == action


@pytest.mark.parametrize('text', [
    'needle holder please',   # "please" must NOT fuzzy-become "release"
    'shop is open',           # "shop" must NOT abort (short words stay exact)
])
def test_fuzzy_verbs_no_false_positives(parser, text):
    assert parser.parse(text).action not in (Action.RELEASE, Action.ABORT)


@pytest.mark.parametrize('text,expected_class', [
    ('awl back', 'awl'),
    ('put the hammer away', 'hammer'),
    ('done with the retractor', 'retractor'),
])
def test_put_back_with_named_tool(parser, text, expected_class):
    intent = parser.parse(text)
    assert intent.action == Action.PUT_BACK
    assert intent.tool_class == expected_class


def test_abort_beats_everything(parser):
    # "stop" must win even when a tool and another verb are in the utterance.
    assert parser.parse('stop, wrong tool, needle holder').action == Action.ABORT


def test_wrong_beats_back(parser):
    # "wrong one, put it back" is a return of the held tool.
    assert parser.parse('wrong one, put it back').action == Action.RETURN


# ── Unknowns ────────────────────────────────────────────────────────

@pytest.mark.parametrize('text', ['', '   ', 'flurble smorp gah'])
def test_unknown(parser, text):
    intent = parser.parse(text)
    assert intent.action == Action.UNKNOWN
    assert not intent.candidates


# ── Normalization ───────────────────────────────────────────────────

def test_normalize_keeps_apostrophes_and_umlauts():
    assert normalize("We're DONE!") == "we're done"
    assert normalize('Zurück, bitte...') == 'zurück bitte'
    assert normalize('porte-aiguille') == 'porte aiguille'
