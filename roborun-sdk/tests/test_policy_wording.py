"""A provider's message never reads as one of Roborun's own errors (the local API's task view, error_info)."""
import importlib.util
import os
import re
from pathlib import Path

import pytest

from roborun_sdk.policy.wording import ROBORUN_CAUSES, WITHHELD, provider_text, roborun_cause

# the runtime's own task view source (its task_view.py), when one is given
RUNTIME_TASK_VIEW = Path(os.environ.get('ROBORUN_RUNTIME_TASK_VIEW') or '/nonexistent/task_view.py')

# a sentence for every pattern of the task view
SAMPLES = [
    "dreamlabs: DreamLabs refused the key: invalid_key", "acme refused this robot's key", 'reason invalid_key',
    'this key cannot run a robot', 'read_only_key', 'too many requests for this key', 'the Manifest job limit is reached',
    'signed with a key this robot does not know', 'unknown signing key', 'it did not pass its signature check',
    'the signature does not verify', 'the signed result does not answer this request',
    'this Roborun version is no longer supported', 'a status this Roborun does not know', 'sdk_upgrade_required',
    'plain http to another computer', 'the Roborun API did not answer in time', 'the Manifest service did not answer in time',
    'the Roborun API is busy or restarting', '/v1/get_action: HTTP 503', 'upstream_error',
    'the Roborun API could not be reached', 'Manifest service at gw.example unreachable', 'behind a captive portal',
    'no camera measured it', 'Face blur could not run', 'the face detector model', 'your rules file failed',
    'Tasks need a depth camera',
]


def test_the_pinned_patterns_are_the_runtime_s():
    if not RUNTIME_TASK_VIEW.exists():
        pytest.skip('ROBORUN_RUNTIME_TASK_VIEW names no runtime task view file')
    spec = importlib.util.spec_from_file_location('roborun_runtime_task_view', RUNTIME_TASK_VIEW)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    assert [(code, p.pattern) for code, p in mod._CAUSES] == list(ROBORUN_CAUSES)


@pytest.mark.parametrize('text', SAMPLES)
def test_every_pattern_is_reworded_and_the_words_stay(text):
    assert roborun_cause(text) is not None, text
    out = provider_text(text)
    assert roborun_cause(out) is None and out != WITHHELD, out
    assert len(out) > len(text) // 2


def test_plain_messages_pass_unchanged():
    for text in ('dreamlabs: DreamLabs refused the request: unknown_checkpoint', 'sway: ValueError: bad rows',
                 'no chunk from acme for 8 s; the arms held'):
        assert provider_text(text) == text


def test_every_cause_is_covered_by_a_sample():
    hit = {roborun_cause(t) for t in SAMPLES}
    assert hit == {code for code, _ in ROBORUN_CAUSES}
    for code, pattern in ROBORUN_CAUSES:
        re.compile(pattern)
