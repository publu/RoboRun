"""Consent before a remote policy provider gets anything: asked before its start() (which may already reach the network).

Stored in its own file, <Roborun home>/policy-consent.json (mode 0600, written atomically), never in robot.yaml. One entry per
(provider, model): {given_at, sends_hash, face_blur, text_version}. An entry counts only while what the provider says it sends
(Needs.sends) and the consent text are unchanged, and while face blur is as it was when consent was given: turning face blur
off asks again, and the text then says the frames go unblurred.

    roborun policy consent                       what has been agreed
    roborun policy consent --revoke PROVIDER     forget it (all of that provider's models; PROVIDER:MODEL for one)

Standard library only.
"""
from __future__ import annotations

import datetime
import hashlib
import json
import os
import sys
from pathlib import Path

from .provider import PolicyRefused

TEXT_VERSION = 1
FILE = 'policy-consent.json'
NEEDED = ("consent needed: run once with `roborun policy run` in a terminal, or pass consent=True")


def path():
    from .. import net
    return net.roborun_home() / FILE


def _load():
    try:
        value = json.loads(path().read_text())
    except (OSError, ValueError):
        return {}
    return value if isinstance(value, dict) else {}


def _save(entries):
    target = path()
    target.parent.mkdir(parents=True, exist_ok=True)
    tmp = target.with_name(target.name + f'.{os.getpid()}.tmp')
    fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, 'w') as f:
        json.dump(entries, f, indent=1, sort_keys=True)
    os.replace(tmp, target)
    try:
        os.chmod(target, 0o600)
    except OSError:
        pass


def key(provider, model):
    return f'{provider}:{model or ""}'


def sends_hash(needs):
    raw = json.dumps(dict(sends=list(needs.sends), text_version=TEXT_VERSION), sort_keys=True).encode()
    return hashlib.sha256(raw).hexdigest()[:16]


def text(provider, needs, face_blur, *, title=None, destination=None):
    """The words shown before anything is sent: built from Needs.sends."""
    who = title or provider
    where = f' ({destination})' if destination else ''
    sends = list(needs.sends) or ['the prompt and the arms\' joint angles']
    blur = ('face-blurred' if face_blur else 'NOT face-blurred: you turned face blur off')
    has_frames = bool(needs.cameras)
    lines = [f'While this run is active, including while the model warms up and before any motion, Roborun sends to {who}{where}:']
    lines += [f'  - {item}' for item in sends]
    if has_frames:
        lines.append(f'  Camera frames are {blur}.')
    lines.append(f'Nothing else is sent to {who}. This run\'s Roborun task record, recording and dataset episode are not sent to '
                 f'{who}; like any task\'s, they go to your Roborun history (roborun up --no-data-sync keeps them on this '
                 'computer). The request timing in log/policy/ never leaves this computer.')
    return '\n'.join(lines)


def stored(provider, model, needs, face_blur):
    """The stored consent that still applies, or None."""
    entry = _load().get(key(provider, model))
    if not isinstance(entry, dict):
        return None
    if entry.get('sends_hash') != sends_hash(needs) or entry.get('text_version') != TEXT_VERSION:
        return None
    if bool(entry.get('face_blur', True)) != bool(face_blur):
        return None                                  # face blur changed since: ask again (the text says what changed)
    return entry


def remember(provider, model, needs, face_blur):
    entries = _load()
    entry = dict(given_at=datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='seconds'),
                 sends_hash=sends_hash(needs), face_blur=bool(face_blur), text_version=TEXT_VERSION)
    entries[key(provider, model)] = entry
    _save(entries)
    return entry


def entries():
    return _load()


def revoke(what):
    """Forget 'provider' (every model) or 'provider:model'. Returns the keys removed."""
    data = _load()
    if ':' in what:
        gone = [k for k in data if k == what]
    else:
        gone = [k for k in data if k.split(':', 1)[0] == what]
    for k in gone:
        data.pop(k)
    if gone:
        _save(data)
    return gone


def decide(provider, model, needs, face_blur, *, consent=None, yes=False, ask=None, words=None, say=print, title=None):
    """{given_at, how} when the run may send; raises PolicyRefused otherwise. Never asked for a local provider.
    consent: True from Python (this run only). yes: --yes (this run only). ask: a function(question) -> answer, only on a
    terminal; a 'y' is stored for later runs of this model."""
    if not needs.remote:
        return dict(given_at=None, how='local')
    if consent is True or yes:
        return dict(given_at=datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='seconds'),
                    how='this run (--yes)' if yes else 'this run (consent=True)')
    entry = stored(provider, model, needs, face_blur)
    if entry is not None:
        return dict(given_at=entry['given_at'], how='stored')
    if ask is None:
        raise PolicyRefused(NEEDED)
    say(words or text(provider, needs, face_blur))
    answer = ask(f'Send this to {title or provider} for this run and later runs of this model? [y/N] ')
    if str(answer or '').strip().lower() not in ('y', 'yes'):
        raise PolicyRefused('not agreed: nothing was sent')
    entry = remember(provider, model, needs, face_blur)
    return dict(given_at=entry['given_at'], how='asked')


def terminal_ask():
    """input() when stdin and stdout are a terminal, else None."""
    if not (sys.stdin and sys.stdin.isatty() and sys.stdout and sys.stdout.isatty()):
        return None
    return input


def show(as_json=False):
    data = _load()
    if as_json:
        print(json.dumps(data, indent=1, sort_keys=True))
    elif not data:
        print('no consent stored')
    else:
        for k, v in sorted(data.items()):
            print(f'{k}: given {v.get("given_at")}' + ('' if v.get('face_blur', True) else ' (frames unblurred)'))
    return 0


def file_path() -> Path:
    return path()
