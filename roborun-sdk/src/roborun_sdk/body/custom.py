"""Your own body class: `roborun up --profile my_rover --body my_rover.py:MyRover` (or `body:` in robot.yaml).

The class is named like an arm driver: a file and a class (`/path/my_rover.py:MyRover`, a relative path is from robot.yaml's
folder) or a module of an installed package (`acme_robots.rover:Rover`). Roborun makes one object of it, with the body's profile
(`profile=` a dict: robot.limits, camera, ...) when its constructor takes a `profile` argument, and every other key of robot.yaml's
`body:` section as keyword arguments. The object is one of:

    an object with .base (a BaseAdapter) and .head (a HeadAdapter), and optionally close()     (like body.sim.BodySim)
    a BaseAdapter that is also the HeadAdapter (one class subclassing both)
    a BaseAdapter with a .head attribute (a HeadAdapter)

The body's description (kind, base, arms, head, speed limits, camera height and lens) always comes from its profile: the
class drives it, the profile describes it.
"""
from __future__ import annotations

import importlib
import importlib.util
import inspect
import sys
from pathlib import Path

from .adapters import BaseAdapter, HeadAdapter


class BodyLoadError(RuntimeError):
    pass


def is_ref(text) -> bool:
    """'my_rover.py:MyRover' or 'package.module:MyRover'."""
    where, sep, cls = str(text).partition(':')
    return bool(sep and where and cls and cls.isidentifier())


def resolve_ref(ref, base_dir=None) -> str:
    """A file path made absolute (relative: from base_dir, else the current folder); a module path as given."""
    ref = str(ref)
    if not is_ref(ref):
        raise BodyLoadError(f'{ref!r}: name your body class as path/to/file.py:ClassName or package.module:ClassName')
    where, _, cls = ref.partition(':')
    if where.endswith('.py'):
        p = Path(where).expanduser()
        if not p.is_absolute():
            p = Path(base_dir or Path.cwd()) / p
        return f'{p.resolve()}:{cls}'
    return ref


def load_class(ref):
    where, _, cls = str(ref).partition(':')
    if where.endswith('.py'):
        path = Path(where).expanduser()
        if not path.is_file():
            raise BodyLoadError(f'no body file at {path}')
        spec = importlib.util.spec_from_file_location(f'roborun_own_body_{path.stem}', path)
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        try:
            spec.loader.exec_module(module)
        except Exception as exc:
            raise BodyLoadError(f'your body file {path} could not be loaded ({type(exc).__name__}: {exc})') from exc
    else:
        try:
            module = importlib.import_module(where)
        except ImportError as exc:
            raise BodyLoadError(f'the module {where} could not be imported ({exc}): is its package installed in Roborun\'s '
                                'Python? (roborun python -m pip install ...)') from exc
        except Exception as exc:
            raise BodyLoadError(f'the module {where} could not be loaded ({type(exc).__name__}: {exc})') from exc
    kind = getattr(module, cls, None)
    if kind is None:
        raise BodyLoadError(f'{where} has no class {cls}')
    if not callable(kind):
        raise BodyLoadError(f'{where}:{cls} is not a class')
    return kind


def default_profile(ref):
    """The profile id a body class names for itself (a class attribute `profile = 'my_rover'`), or None."""
    value = getattr(load_class(ref), 'profile', None)
    return value if isinstance(value, str) else None


def split(obj):
    """(base, head) of what a body class made; BodyLoadError when it is not a body."""
    base = obj if isinstance(obj, BaseAdapter) else getattr(obj, 'base', None)
    if isinstance(obj, BaseAdapter) and isinstance(obj, HeadAdapter):
        head = obj
    else:
        head = getattr(obj, 'head', None)
    if not isinstance(base, BaseAdapter):
        raise BodyLoadError(f'{type(obj).__name__} is not a body: give it a .base that is a roborun_sdk.body.BaseAdapter '
                            '(command, stop, state), or make the class a BaseAdapter')
    if not isinstance(head, HeadAdapter):
        raise BodyLoadError(f'{type(obj).__name__} has no head camera: give it a .head that is a roborun_sdk.body.HeadAdapter '
                            '(command, state, frame; dof = 0 for a fixed camera)')
    return base, head


def open_custom(ref, profile, options=None):
    """(base, head, close, warnings): the user's body class made with this profile (a dict) and options."""
    kind = load_class(ref)
    options = dict(options or {})
    try:
        params = inspect.signature(kind).parameters
    except (TypeError, ValueError):
        params = {}
    open_kw = any(p.kind == p.VAR_KEYWORD for p in params.values())
    kw = {k: v for k, v in options.items() if open_kw or k in params}
    unused = sorted(set(options) - set(kw))
    if 'profile' in params or open_kw:
        kw['profile'] = profile
    try:
        obj = kind(**kw)
    except Exception as exc:
        raise BodyLoadError(f'{ref}: could not start your body ({type(exc).__name__}: {exc})') from exc
    base, head = split(obj)
    warnings = [f'body: setting {k!r} is not taken by {kind.__name__}' for k in unused]
    robot = profile['robot']
    if getattr(base, 'kind', robot['base']) != robot['base']:
        warnings.append(f"body: {type(base).__name__}.kind is {base.kind!r}, the profile says base {robot['base']!r}; "
                        'the profile is what the motion service is told')
    if getattr(head, 'dof', robot['head_dof']) != robot['head_dof']:
        warnings.append(f"body: {type(head).__name__}.dof is {head.dof}, the profile says head_dof {robot['head_dof']}; "
                        'the head is driven as the profile says')

    def close():
        for part in ([obj] if callable(getattr(obj, 'close', None)) and obj is not base and obj is not head else []) + [base, head]:
            try:
                part.close()
            except Exception:
                pass
    return base, head, close, warnings
