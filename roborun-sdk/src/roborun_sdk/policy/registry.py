"""Finding a policy provider by name: a built-in, an entry point, or your own class in a file or module.

    hold                      built in: keeps the arms where they are (a self-test; it moves nothing)
    replay                    built in: plays back the `action` rows of a recorded episode (an .npz or a LeRobot dataset)
    dreamlabs                 a DreamLabs model (Dream-1 or your fine-tune), with pip install 'roborun-sdk[dreamlabs]'
    my_policy.py:MyPolicy     a class in a file (the path as given, else under the Roborun home)
    mypackage.policies:Mine   a class in an importable module
    <name>                    a package's entry point in the group `roborun.policies`

The class is called with the run's options as keyword arguments (`--option k=v`); options it does not take are refused before
anything is sent or moves. Standard library only.
"""
from __future__ import annotations

import importlib
import importlib.metadata
import importlib.util
import inspect
import re
import sys
from dataclasses import dataclass
from pathlib import Path

from .provider import PolicyRefused

GROUP = 'roborun.policies'
# the same form as a custom camera class (roborun_sdk.cameras.CUSTOM_SOURCE): file.py:Class or module:Class
CUSTOM = re.compile(r'(.+\.py|[A-Za-z_][\w.]*):[A-Za-z_]\w*')
# name -> (module, class, the extra it needs or None, the top-level module that extra brings, the distribution whose version
# is shown). The module is imported only when the provider is used; providers() only looks the modules up.
BUILTIN = {
    'hold': ('roborun_sdk.policy.builtin', 'HoldProvider', None, None, None),
    'replay': ('roborun_sdk.policy.builtin', 'ReplayProvider', None, None, None),
    'dreamlabs': ('roborun_sdk.policy.dreamlabs', 'DreamLabsProvider', 'dreamlabs', 'dream', 'dreamlabs-client'),
}


@dataclass(frozen=True)
class ProviderEntry:
    name: str
    source: str                 # 'built-in' | 'entry point' | 'file' | 'module'
    version: str | None
    available: bool
    extra: str | None           # the roborun-sdk extra it needs, if any
    note: str | None = None


def _sdk_version():
    try:
        return importlib.metadata.version('roborun-sdk')
    except importlib.metadata.PackageNotFoundError:
        return None


def _builtin_entry(name):
    module, cls, extra, needs, dist = BUILTIN[name]
    if importlib.util.find_spec(module) is None:
        return ProviderEntry(name, 'built-in', None, False, extra, 'not part of this build')
    if needs is not None and importlib.util.find_spec(needs) is None:
        return ProviderEntry(name, 'built-in', None, False, extra, f"needs pip install 'roborun-sdk[{extra}]'")
    if dist is not None:
        try:
            return ProviderEntry(name, 'built-in', f'{dist} {importlib.metadata.version(dist)}', True, extra)
        except importlib.metadata.PackageNotFoundError:
            return ProviderEntry(name, 'built-in', None, True, extra)
    return ProviderEntry(name, 'built-in', _sdk_version(), True, extra)


def entry_points():
    try:
        return {ep.name: ep for ep in importlib.metadata.entry_points(group=GROUP)}
    except Exception:
        return {}


def providers():
    """[ProviderEntry] of every provider this computer has: the built-ins, then the installed entry points."""
    out = [_builtin_entry(name) for name in BUILTIN]
    for name, ep in sorted(entry_points().items()):
        if name in BUILTIN:
            continue
        dist = getattr(ep, 'dist', None)
        out.append(ProviderEntry(name, 'entry point', f'{dist.name} {dist.version}' if dist else None, True, None,
                                 ep.value))
    return out


def _load_file(where, name):
    candidates = [Path(where).expanduser()]
    try:
        from .. import net
        candidates.append(net.roborun_home() / where)
    except Exception:
        pass
    path = next((c for c in candidates if c.is_file()), None)
    if path is None:
        raise PolicyRefused(f'policy {where}:{name}: there is no file {where}')
    spec = importlib.util.spec_from_file_location(f'roborun_own_policy_{path.stem}', path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def resolve(name):
    """The provider class for a name (see the module docstring). Raises PolicyRefused with the reason."""
    if not isinstance(name, str) or not name:
        raise PolicyRefused('name a policy provider: hold, replay, dreamlabs, file.py:Class, module:Class or an installed name')
    try:
        if name in BUILTIN:
            module, cls = BUILTIN[name][:2]
            if importlib.util.find_spec(module) is None:
                raise PolicyRefused(f'the {name} provider is not part of this build of roborun-sdk')
            return getattr(importlib.import_module(module), cls)
        eps = entry_points()
        if name in eps:
            return eps[name].load()
        if CUSTOM.fullmatch(name):
            where, _, cls = name.rpartition(':')
            if '.' not in sys.path:
                sys.path.insert(0, '.')
            module = _load_file(where, cls) if where.endswith('.py') else importlib.import_module(where)
            kind = getattr(module, cls, None)
            if not inspect.isclass(kind):
                raise PolicyRefused(f'policy {name}: {where} has no class {cls}')
            return kind
    except PolicyRefused:
        raise
    except ModuleNotFoundError as exc:
        from .._optional import explain
        raise PolicyRefused(explain(exc) or f'policy {name}: could not load it ({exc})') from exc
    except Exception as exc:
        raise PolicyRefused(f'policy {name}: could not load it ({type(exc).__name__}: {exc})') from exc
    raise PolicyRefused(f'unknown policy provider {name!r}: use hold, replay, dreamlabs, an installed entry point '
                        f'({GROUP}), file.py:Class or module:Class')


def create(name, options=None):
    """An instance of the provider `name` (a provider object is returned as it is), with `options` as keyword arguments."""
    if not isinstance(name, str):
        if options:
            raise PolicyRefused('options go to a provider by name; this one is already made')
        return name
    kind = resolve(name)
    options = dict(options or {})
    try:
        params = inspect.signature(kind).parameters
    except (TypeError, ValueError):
        params = {}
    takes_any = any(p.kind == p.VAR_KEYWORD for p in params.values())
    unknown = sorted(k for k in options if not takes_any and k not in params)
    if unknown:
        raise PolicyRefused(f'the {name} provider does not take the option{"s" if len(unknown) > 1 else ""} '
                            f'{", ".join(unknown)}; nothing was sent')
    try:
        provider = kind(**options)
    except TypeError as exc:
        raise PolicyRefused(f'the {name} provider refused its options: {exc}; nothing was sent') from exc
    for method in ('needs', 'act'):
        if not callable(getattr(provider, method, None)):
            raise PolicyRefused(f'{name} is not a policy provider: it has no {method}() method')
    if not getattr(provider, 'name', None) or getattr(provider, 'name', None) == 'provider':
        try:
            provider.name = name
        except Exception:
            pass
    return provider


def parse_options(pairs):
    """['k=v', ...] -> {k: v}, with true/false, numbers and JSON values understood."""
    import json
    out = {}
    for pair in pairs or ():
        key, sep, value = str(pair).partition('=')
        key = key.strip()
        if not sep or not re.fullmatch(r'[A-Za-z_]\w*', key):
            raise PolicyRefused(f'--option {pair!r}: write it as key=value')
        text = value.strip()
        if text.lower() in ('true', 'false'):
            out[key] = text.lower() == 'true'
            continue
        try:
            out[key] = json.loads(text)
        except ValueError:
            out[key] = text
    return out
