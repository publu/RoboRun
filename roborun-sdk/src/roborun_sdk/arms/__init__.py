"""The arms Roborun can drive. Each arm is one file in data/arms/<id>.json:

{"id", "label", "driver": "module:Class", "motors", "transport": "can|serial|ethernet|sdk", "install": "<pip extra>",
 "status": "supported|beta", "profile": "<runtime profile or null>", "home": [...], "limits": [[lo, hi], ...], "options": {}}

`status` is honest: "supported" has run on real hardware, "beta" has only run against a simulated bus or the vendor's
simulator, "bring-your-own" has no built-in driver (driver null): it runs with the user's own ArmAdapter. Any arm works
that way, listed here or not: roborun up --left my_arm.py:MyArm@ADDRESS --rig my_rig.json
"""
import importlib
import importlib.util
import inspect
import json
import sys
from importlib import resources
from pathlib import Path

from ..errors import ArmLoadError     # a RoborunError and a SystemExit: catchable by programs, a plain message in the CLI


def catalog() -> dict:
    out = {}
    folder = resources.files('roborun_sdk') / 'data' / 'arms'
    if not folder.is_dir():
        return out
    for item in folder.iterdir():
        if item.name.endswith('.json'):
            row = json.loads(item.read_text())
            out[row['id']] = row
    return dict(sorted(out.items()))


def parse_spec(text: str):
    """'yam@slcan://SERIAL' -> ('yam', 'slcan://SERIAL'); 'yam' -> ('yam', None)."""
    arm, _, bus = str(text).partition('@')
    return arm, bus or None


def is_own_driver(arm: str) -> bool:
    """'my_arm.py:MyArm' or 'package.module:MyArm': a driver the user wrote."""
    return ':' in str(arm)


def _load_own(ref: str):
    where, _, cls = ref.partition(':')
    if where.endswith('.py'):
        path = Path(where).expanduser()
        if not path.is_file():
            raise ArmLoadError(f'no driver file at {path}')
        spec = importlib.util.spec_from_file_location(f'roborun_own_arm_{path.stem}', path)
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        try:
            spec.loader.exec_module(module)
        except Exception as exc:
            raise ArmLoadError(f'your driver {path} could not be loaded ({type(exc).__name__}: {exc})') from exc
    else:
        module = importlib.import_module(where)
    kind = getattr(module, cls, None)
    if kind is None:
        raise ArmLoadError(f'{where} has no class {cls}')
    return kind


def open_arm(arm: str, side: str, bus=None, connect=True, **overrides):
    """An arm from the catalog or the user's own file, with the settings from robot.yaml. Every driver finds all of them in
    `.settings`; a setting nothing uses is named in `.setting_warnings`, and never stops the arm."""
    from ..embodiment import ArmAdapter
    from .base import StreamedArm
    kw = {}
    if is_own_driver(arm):
        kind = _load_own(arm)
    else:
        row = catalog().get(arm)
        if row is None:
            raise ArmLoadError(f'unknown arm {arm!r}: `roborun hw arms` lists them, or pass your own driver as my_arm.py:MyArm')
        if not row.get('driver'):
            raise ArmLoadError(f'{row["label"]} has no built-in driver yet: write a driver for it on StreamedArm (roborun new my-robot) and pass my_arm.py:MyArm')
        from .packages import ensure
        ensure(row.get('install'), row['label'])
        module, _, cls = row['driver'].partition(':')
        try:
            kind = getattr(importlib.import_module(module), cls)
        except ImportError as exc:
            raise ArmLoadError(f'{row["label"]} needs {exc.name}, which is not available on this computer'
                             + (f': {row["note"]}' if row.get('note') else ''))
        kw = dict(row.get('options') or {})
        for key in ('home', 'limits'):
            if row.get(key) is not None: kw[key] = row[key]
    kw.update(overrides)
    if isinstance(kind, type) and not issubclass(kind, ArmAdapter) and callable(getattr(kind, 'read', None)) and callable(getattr(kind, 'command', None)):
        kind = type(kind.__name__, (kind, ArmAdapter), {})    # a small class with read() and command() is enough
    if not (isinstance(kind, type) and issubclass(kind, ArmAdapter)):
        raise ArmLoadError(f'{arm} is not an ArmAdapter: a driver needs read() and command() methods')
    unused = []
    if issubclass(kind, StreamedArm):
        kw.update(bus=bus, side=side)
    else:                                                 # a plain ArmAdapter takes only what its constructor names
        accepted = inspect.signature(kind).parameters
        open_kw = any(p.kind == p.VAR_KEYWORD for p in accepted.values())
        if overrides.get('dry_run') and not getattr(kind, 'supports_dry_run', False):
            raise ArmLoadError(f'{arm} cannot do a dry run: set supports_dry_run = True on it once connect() enables nothing '
                             'when it is given dry_run=True')
        unused = sorted(k for k in overrides if not open_kw and k not in accepted)
        kw = {k: v for k, v in dict(kw, bus=bus, side=side).items() if open_kw or k in accepted}
    obj = kind(**kw)
    obj.settings = dict(overrides)
    unused += [k for k in getattr(obj, 'unused_options', ()) if k in overrides]
    warnings = [f'{side} arm: setting {name!r} is not used by {arm}; it is available to the driver as .settings' for name in sorted(set(unused))]
    obj.setting_warnings = list(getattr(obj, 'setting_warnings', ())) + warnings
    for text in warnings:
        print(json.dumps(dict(event='arm_setting_unused', arm=side, message=text)), flush=True)
    if connect:
        obj.connect()
    return obj
