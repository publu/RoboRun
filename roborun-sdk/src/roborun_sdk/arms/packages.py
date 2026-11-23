"""The packages a particular arm or camera needs, installed the first time it is used, so nobody picks them by hand."""
import json
import subprocess
import sys
from importlib import metadata

from packaging.requirements import Requirement

DISTRIBUTION = 'roborun-sdk'


def requirements(extra: str) -> list:
    """What `extra` adds on this computer, as Requirement objects without their markers."""
    try:
        declared = metadata.requires(DISTRIBUTION) or []
    except metadata.PackageNotFoundError:                 # a source checkout manages its own environment
        return []
    out = []
    for text in declared:
        req = Requirement(text)
        if req.marker and req.marker.evaluate({'extra': extra}) and not req.marker.evaluate({'extra': ''}):
            req.marker = None
            out.append(req)
    return out


def missing(extra: str) -> list:
    out = []
    for req in requirements(extra):
        try:
            have = metadata.version(req.name)
        except metadata.PackageNotFoundError:
            out.append(str(req))
            continue
        if not req.specifier.contains(have, prereleases=True):
            out.append(str(req))
    return out


def ensure(extra, label=None):
    """Install what `extra` still needs into this Python. A package with no build for this computer is a clear error."""
    if not extra:
        return
    wanted = missing(extra)
    if not wanted:
        return
    print(f'Installing support for {label or extra}...', flush=True)
    done = subprocess.run([sys.executable, '-m', 'pip', 'install', '-q', *wanted], capture_output=True, text=True)
    if done.returncode:
        raise SystemExit(f'could not install what {label or extra} needs on this computer ({", ".join(wanted)}):\n'
                         + done.stderr.strip()[-800:])


def prepare(hardware):
    """Everything a hardware section will need, before anything is started in the background."""
    from . import catalog, is_own_driver
    rows = catalog()
    for side in ('left', 'right'):
        arm = (hardware or {}).get(side)
        if arm and not is_own_driver(arm['arm']):
            row = rows.get(arm['arm']) or {}
            ensure(row.get('install'), row.get('label'))
    if (hardware or {}).get('cameras'):
        ensure('cameras', 'cameras')


def prepare_for_up(a):
    from ..robot import CONFIG
    from .hardware import config_from_args
    old = None
    if CONFIG.exists():
        old = json.loads(CONFIG.read_text()).get('hardware')
    prepare(config_from_args(a, old))
