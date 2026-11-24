"""A provider's words never read as Roborun's own errors.

The robot's local API (its task view) turns some phrases in a failed task's error into Roborun's own causes: the Roborun API
refused this robot's key, the Roborun API is down, a reply failed its signature check... and then tells the user to run
`roborun login` or `roborun upgrade`. A policy provider's message ("DreamLabs refused the key: invalid_key", "/v1/get_action:
HTTP 503") must not match them. provider_text() rewords any match. ROBORUN_CAUSES is a copy of the task view's patterns;
tests/test_policy_wording.py checks the copy against the runtime's when the runtime's source is beside the packages."""
from __future__ import annotations

import re

ROBORUN_CAUSES = (
    ('key_revoked', r"refused (?:this robot's|the) key|invalid_key"),
    ('key_read_only', r'key cannot run a robot|read_only_key'),
    ('rate_limited', r'too many requests for this key|Manifest job limit is reached'),
    ('signing_key_unknown', r'key this robot does not know|unknown signing key'),
    ('signature_invalid', r'did not pass its signature check|signature does not verify|signed result does not answer this request'),
    ('sdk_upgrade_required', r'Roborun version is no longer supported|status this Roborun does not know|sdk_upgrade_required'),
    ('insecure_url', r'plain http to another computer'),
    ('service_timeout', r'(?:Roborun API|Manifest service) did not answer in time'),
    ('service_unavailable', r'Roborun API is busy or restarting|/v1/[\w/.-]+: HTTP 50[0234]|upstream_error'),
    ('network_offline', r'Roborun API could not be reached|Manifest service at \S+ unreachable|captive portal'),
    ('object_not_measured', r'no camera measured'),
    ('face_blur_unavailable', r'Face blur could not run|face detector model'),
    ('rules_failed', r'rules file failed'),
    ('no_depth_camera', r'Tasks need a depth camera'),
)
_PATTERNS = tuple(re.compile(p, re.I) for _, p in ROBORUN_CAUSES)

# phrase -> the same meaning in words the task view does not claim (applied to a matched span only)
_REWORD = (
    (r"refused (?:this robot's|the) key", 'did not accept the key'),
    (r'key cannot run a robot', 'key may not drive a robot'),
    (r'too many requests for this key', 'too many requests (provider key)'),
    (r'key this robot does not know', 'key unknown here'),
    (r'unknown signing key', 'signing key not known'),
    (r'did not pass its signature check', 'failed its signature check'),
    (r'signature does not verify', 'signature fails to verify'),
    (r'signed result does not answer this request', 'signed result is for another request'),
    (r'Roborun version is no longer supported', 'version is no longer supported'),
    (r'status this Roborun does not know', 'status unknown to this Roborun'),
    (r'plain http to another computer', 'unencrypted http to another computer'),
    (r'(?:Roborun API|Manifest service)', 'provider service'),
    (r': HTTP (50\d)', r': HTTP status \1'),
    (r'captive portal', 'captive-portal network'),
    (r'no camera measured', 'no camera has measured'),
    (r'Face blur could not run', 'face blur failed'),
    (r'face detector model', 'face-detector model'),
    (r'rules file failed', 'rules file has failed'),
    (r'Tasks need a depth camera', 'tasks require a depth camera'),
    (r'Manifest job limit', 'provider job limit'),
    (r'_', ' '),                                           # invalid_key, read_only_key, upstream_error, ...
)
WITHHELD = 'the provider reported an error (its own words are in the run\'s local log, log/policy/<run id>/)'


def _reword(span):
    for pattern, repl in _REWORD:
        span = re.sub(pattern, repl, span, flags=re.I)
    return span


def roborun_cause(text):
    """The task view's cause code this text would be read as, or None."""
    for (code, _), pattern in zip(ROBORUN_CAUSES, _PATTERNS):
        if pattern.search(str(text or '')):
            return code
    return None


def provider_text(text):
    """text, reworded wherever the task view would read it as one of Roborun's own errors."""
    out = str(text if text is not None else '')
    for _ in range(4):
        if roborun_cause(out) is None:
            return out
        for pattern in _PATTERNS:
            out = pattern.sub(lambda m: _reword(m.group(0)), out)
    return out if roborun_cause(out) is None else WITHHELD
