"""The stable Python surface of roborun-sdk, and a check that it is only ever added to within a series.

docs/BUILDING.md ("Versions and upgrades") promises that within a series (every 0.1.* release) these are added to, never
renamed or removed: roborun_sdk.client, roborun_sdk.errors.RoborunError, StreamedArm and its *_hardware methods, open_arm,
the rules check(side, positions_rad, gripper, state), the camera source class and the camera card functions
(roborun_sdk.camera_card). This tool reads them from the source (or
from a released wheel) with the ast module, never importing anything, and compares them with tools/stable_surface.json:
the surface of the release named there. Every name that was removed or whose signature changed since that release must be
named exactly, in backticks, in an entry under a **Removed** or **Changed** heading of CHANGELOG.md, in a section newer than
that release ("Unreleased" counts). A mention elsewhere (under **Added**, or of a longer name) does not count. New names
are fine.

    python tools/stable_surface.py                       check the source against the snapshot (exit 1 on a break)
    python tools/stable_surface.py --show                print the surface of the source
    python tools/stable_surface.py --old A.whl --new B   compare any two: a wheel, an unpacked wheel or a source tree
    python tools/stable_surface.py --update --release 0.1.<time>
                                                         after a release: the snapshot becomes that release's surface
                                                         (only when the check passes)
"""
from __future__ import annotations

import argparse
import ast
import json
import re
import sys
import zipfile
from pathlib import Path

SDK = Path(__file__).resolve().parents[1]
SNAPSHOT = SDK / 'tools' / 'stable_surface.json'
CHANGELOG = SDK / 'CHANGELOG.md'

# What BUILDING.md's stable column names, by file (relative to roborun_sdk/). '*' = the module's __all__ and, for each of
# its classes, the public methods and (for a TypedDict) the fields.
STABLE = {
    'client.py': '*',
    'errors.py': ('RoborunError',),
    'arms/__init__.py': ('open_arm',),
    'arms/base.py': ('StreamedArm',),             # its __init__ and *_hardware methods (what a driver implements)
    'camera_card.py': ('check_config', 'derive', 'body_card', 'describe', 'validate', 'outbound', 'normalise_intrinsics',
                       'nominal_pinhole'),        # camera cards (roborun.camera/1)
}
PUBLIC_NAMES = ('Robot', 'RobotError', 'RobotUnavailable', 'Task', 'StopResult', 'StopStatus', 'RoborunError', 'StreamedArm',
                'open_arm')                     # the names BUILDING.md's stable column spells out


# ---------------------------------------------------------------------------------------------------- reading a tree
def _files(where):
    """{path relative to roborun_sdk/: source} for the files STABLE names, from a wheel, an unpacked wheel or a source tree."""
    where = Path(where)
    out = {}
    if where.is_file() and where.suffix == '.whl':
        with zipfile.ZipFile(where) as z:
            for rel in STABLE:
                try:
                    out[rel] = z.read(f'roborun_sdk/{rel}').decode()
                except KeyError:                # a module a release before it did not have: nothing of it to compare
                    pass
        return out
    for root in (where / 'src' / 'roborun_sdk', where / 'roborun_sdk', where):
        if (root / 'client.py').is_file():
            break
    else:
        raise SystemExit(f'{where}: no roborun_sdk/client.py (give a wheel, an unpacked wheel or the roborun-sdk folder)')
    for rel in STABLE:
        if (root / rel).is_file():
            out[rel] = (root / rel).read_text()
    return out


def _signature(fn):
    returns = f' -> {ast.unparse(fn.returns)}' if fn.returns else ''
    return f'({ast.unparse(fn.args)}){returns}'


def _class(cls, mod, out, methods=None):
    bases = [ast.unparse(b) for b in cls.bases]
    out[f'{mod}:{cls.name}'] = f"class({', '.join(bases)})"
    for node in cls.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and (not node.name.startswith('_') or node.name == '__init__'):
            if methods is None or methods(node.name):
                out[f'{mod}:{cls.name}.{node.name}'] = _signature(node)
        elif 'TypedDict' in bases and isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            out[f'{mod}:{cls.name}.{node.target.id}'] = f'field: {ast.unparse(node.annotation)}'


def _top(tree):
    return {n.name: n for n in tree.body if isinstance(n, (ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef))}


def surface(where):
    """{'roborun_sdk.<module>:<name>': signature} for everything the stable column covers."""
    files = _files(where)
    out = {}
    for rel, wanted in STABLE.items():
        if rel not in files:
            continue
        mod = 'roborun_sdk.' + rel[:-3].replace('/__init__', '').replace('/', '.')
        tree = ast.parse(files[rel])
        defs = _top(tree)
        if wanted == '*':
            names = []
            for node in tree.body:
                if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == '__all__' for t in node.targets):
                    names = list(ast.literal_eval(node.value))
            values = {t.id: node.value for node in tree.body if isinstance(node, ast.Assign) for t in node.targets
                      if isinstance(t, ast.Name)}
            for name in names:
                if isinstance(defs.get(name), ast.ClassDef):
                    _class(defs[name], mod, out)
                elif name in defs:
                    out[f'{mod}:{name}'] = _signature(defs[name])
                elif name in values:
                    out[f'{mod}:{name}'] = f'= {ast.unparse(values[name])}'
            continue
        for name in wanted:
            node = defs.get(name)
            if isinstance(node, ast.ClassDef):
                only = (lambda m: m == '__init__' or m.endswith('_hardware')) if name == 'StreamedArm' else None
                _class(node, mod, out, only)
            elif node is not None:
                out[f'{mod}:{name}'] = _signature(node)
        if rel == 'arms/base.py' and 'StreamedArm' in defs:
            out.update(_rules_contract(defs['StreamedArm']))
    out.update(_camera_contract(where))
    return dict(sorted(out.items()))


def _rules_contract(streamed):
    """How StreamedArm calls the rules file's check(): its positional argument count and keyword names."""
    for node in ast.walk(streamed):
        if isinstance(node, ast.Call) and ast.unparse(node.func) == 'self._rule':
            return {'roborun_sdk.arms.base:rules check()': f'{len(node.args)} positional'
                    + ''.join(f', {k.arg}=' for k in node.keywords)}
    return {}


def _camera_contract(where):
    """The camera source class: what roborun_sdk.cameras.CustomSource offers its __init__ and calls on it."""
    where = Path(where)
    try:
        if where.is_file():
            with zipfile.ZipFile(where) as z:
                text = z.read('roborun_sdk/cameras.py').decode()
        else:
            root = next(r for r in (where / 'src' / 'roborun_sdk', where / 'roborun_sdk', where) if (r / 'cameras.py').is_file())
            text = (root / 'cameras.py').read_text()
    except (KeyError, StopIteration):
        return {}
    cls = _top(ast.parse(text)).get('CustomSource')
    if cls is None:
        return {}
    calls, offered = set(), set()
    for node in ast.walk(cls):
        if isinstance(node, ast.Attribute) and ast.unparse(node.value) == 'self.camera':
            calls.add(node.attr)
        if isinstance(node, ast.Call) and ast.unparse(node.func) == 'getattr' and len(node.args) >= 2 \
                and ast.unparse(node.args[0]) == 'self.camera' and isinstance(node.args[1], ast.Constant):
            calls.add(node.args[1].value)
        if isinstance(node, ast.Call) and ast.unparse(node.func) == 'dict':
            for k in node.keywords:
                if k.arg == 'role':
                    offered.add('role')
                    for c in ast.walk(node):
                        if isinstance(c, ast.Tuple):
                            offered.update(e.value for e in c.elts if isinstance(e, ast.Constant))
    return {'roborun_sdk.cameras:camera source class': f"__init__ may take {', '.join(sorted(offered))}; "
                                                       f"called: {', '.join(sorted(calls))}"}


# ---------------------------------------------------------------------------------------------------- the changelog
def _key(version):
    return tuple(int(p) for p in version.split('.'))


def changelog_since(text, release):
    """The CHANGELOG.md text of every section newer than `release` ('## Unreleased' and '## 0.1.<time> ...' headings)."""
    out, keep = [], True
    for line in text.splitlines():
        m = re.match(r'^## (?:\[)?(Unreleased|\d+\.\d+\.\d+)', line)
        if m:
            keep = m.group(1) == 'Unreleased' or _key(m.group(1)) > _key(release)
        if keep:
            out.append(line)
    return '\n'.join(out)


def short(key):
    """'roborun_sdk.client:Robot.clip' -> 'Robot.clip': how a changelog names it."""
    return key.split(':', 1)[1]


def compare(old, new):
    removed = sorted(k for k in old if k not in new)
    changed = sorted(k for k in old if k in new and old[k] != new[k])
    added = sorted(k for k in new if k not in old)
    return removed, changed, added


BREAK_HEADINGS = ('Removed', 'Changed')


def break_entries(text):
    """The lines of `text` (CHANGELOG.md sections) under a **Removed** or **Changed** heading, up to the next heading."""
    out, under = [], False
    for line in text.splitlines():
        if line.startswith('## '):
            under = False
            continue
        m = re.match(r'^\*\*([^*]+)\*\*', line)
        if m:
            under = m.group(1).split()[0] in BREAK_HEADINGS
            continue
        if under:
            out.append(line)
    return '\n'.join(out)


def names(name, text):
    """True when `text` names exactly `name` in backticks: `Robot.stop` or `Robot.stop(...)`, not `Robot.stop_task`."""
    return re.search('`' + re.escape(name) + r'(?=[`(])', text) is not None


def check(current, snapshot, changelog_text):
    """[(key, 'removed' | 'changed', old, new)] that CHANGELOG.md does not name, since the snapshot's release.

    A name is excused only by an entry under a **Removed** or **Changed** heading of a section newer than the snapshot's
    release that names it exactly in backticks; a mention under **Added**, in prose or of a longer name does not count."""
    entries = break_entries(changelog_since(changelog_text, snapshot['release']))
    removed, changed, _ = compare(snapshot['surface'], current)
    missing = []
    for kind, keys in (('removed', removed), ('changed', changed)):
        for k in keys:
            if not names(short(k), entries):
                missing.append((k, kind, snapshot['surface'][k], current.get(k)))
    return missing


def report(old, new, out=print):
    removed, changed, added = compare(old, new)
    for k in removed:
        out(f'removed  {k}  {old[k]}')
    for k in changed:
        out(f'changed  {k}\n           was {old[k]}\n           now {new[k]}')
    for k in added:
        out(f'added    {k}  {new[k]}')
    return removed, changed, added


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--show', action='store_true', help='print the surface of --new (default: this source)')
    ap.add_argument('--old', help='a wheel, an unpacked wheel or a source tree (default: the snapshot)')
    ap.add_argument('--new', default=str(SDK), help='the same (default: this source)')
    ap.add_argument('--update', action='store_true', help='write the snapshot from --new, once the check passes')
    ap.add_argument('--release', help='with --update: the release the source now is (0.1.<time>)')
    a = ap.parse_args(argv)
    current = surface(a.new)
    if a.show:
        for k, v in current.items():
            print(f'{k}  {v}')
        return 0
    if a.old:
        removed, changed, _ = report(surface(a.old), current)
        return 1 if removed or changed else 0
    snapshot = json.loads(SNAPSHOT.read_text()) if SNAPSHOT.is_file() else None
    if snapshot is None and not a.update:
        raise SystemExit(f'no {SNAPSHOT.name}: write it from the series\' first release with --update --release V --new WHEEL')
    missing = check(current, snapshot, CHANGELOG.read_text() if CHANGELOG.is_file() else '') if snapshot else []
    for k, kind, was, now in missing:
        print(f'{kind} since {snapshot["release"]} and not in CHANGELOG.md: `{short(k)}`  was {was}' + (f'  now {now}' if now else ''))
    if missing:
        print('A stable name may only be removed or changed with a CHANGELOG.md entry naming it (in backticks) under **Removed** '
              'or **Changed**; within a series '
              'this breaks the promise in docs/BUILDING.md "Versions and upgrades", so prefer adding beside it.')
        return 1
    if a.update:
        if not a.release or not re.fullmatch(r'\d+\.\d+\.\d{14}', a.release):
            raise SystemExit('--update needs --release 0.1.<YYYYMMDDHHMMSS>: the release this source was published as')
        SNAPSHOT.write_text(json.dumps(dict(release=a.release, surface=current), indent=1) + '\n')
        print(f'snapshot: {len(current)} names, release {a.release}')
        return 0
    removed, changed, added = compare(snapshot['surface'], current)
    print(f'stable surface: {len(current)} names; since {snapshot["release"]}: {len(removed)} removed, {len(changed)} changed '
          f'(each in CHANGELOG.md), {len(added)} added')
    return 0


if __name__ == '__main__':
    sys.exit(main())
