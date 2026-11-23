"""What a developer building on Roborun needs from the command line:

    roborun --version            the SDK, the installed build and the robot runtime
    roborun docs                 where the agent guide, the README, the building guide, the hosted API notes and the examples are
    roborun docs agents          print one of them (agents, readme, building, hosted-api, examples); `agents` is the one guide
                                 an LLM assistant needs to run Roborun for someone (install to uninstall, every route and error)
    roborun docs --examples DIR  copy the examples into DIR, ready to run
    roborun python FILE [ARGS]   run a Python file with Roborun's own Python, where roborun_sdk is installed

The docs and examples ship inside the package (roborun_sdk/_docs, added when the wheel is built); from a source checkout they
are read from the checkout itself.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
from pathlib import Path

from . import __version__

LOCAL_DOCS = 'http://127.0.0.1:8120/v1/docs'
RELEASES_INDEX = 'https://roborun.app/dl/simple/'          # every Roborun release, as a pip / uv find-links page
DOCS = {'agents': 'docs/AGENTS.md', 'readme': 'README.md', 'building': 'docs/BUILDING.md', 'hosted-api': 'docs/HOSTED_API.md', 'examples': 'examples/README.md'}


def docs_root() -> Path | None:
    """The folder that holds README.md, docs/ and examples/: inside the installed package, or the source checkout."""
    here = Path(__file__).resolve().parent
    for root in (here / '_docs', here.parents[1]):
        if (root / 'docs' / 'BUILDING.md').is_file():
            return root
    return None


def _config_home() -> Path:
    return Path(os.environ.get('ROBORUN_HOME', Path.home() / '.config' / 'roborun'))


def runtime_root() -> Path:
    if os.environ.get('MANIFEST_DREAMAPI'):
        return Path(os.environ['MANIFEST_DREAMAPI'])
    try:
        return Path(json.loads((_config_home() / 'robot.json').read_text())['manifest']).expanduser()
    except (OSError, ValueError, KeyError, TypeError):
        return Path.home() / 'roborun' / 'robot' / 'runtime'


def version_info() -> dict:
    """{sdk, build, runtime, python}: sdk is the PEP 440 version (0.1.<UTC YYYYMMDDHHMMSS> for a release, 0.1.0 from source),
    build the release's build id (None from a source checkout), runtime the robot runtime `roborun up` downloaded (None before
    the first `roborun up`)."""
    from .net import build as release_build
    build = release_build()
    runtime = None
    try:
        runtime = json.loads((runtime_root() / 'RUNTIME.json').read_text()).get('version')
    except (OSError, ValueError, AttributeError):
        pass
    return {'sdk': __version__, 'build': build, 'runtime': runtime, 'python': sys.executable}


def version_text() -> str:
    v = version_info()
    pin = ''
    if v['build'] and not v['sdk'].endswith('.0'):          # a release (0.1.<time>); a source checkout's 0.1.0 is never published
        pin = f"\npin it: roborun-sdk=={v['sdk']}  (pip install --find-links {RELEASES_INDEX} roborun-sdk=={v['sdk']})"
    return (f"roborun-sdk {v['sdk']}" + (f" (build {v['build']})" if v['build'] else ' (source checkout: no release build)')
            + f"\nrobot runtime {v['runtime'] or 'not installed yet (roborun up installs it)'}\npython {v['python']}" + pin)


class VersionAction(argparse.Action):
    def __init__(self, option_strings, dest, **kw):
        super().__init__(option_strings, dest, nargs=0, help='print the SDK, build and robot runtime versions')

    def __call__(self, parser, namespace, values, option_string=None):
        print(version_text())
        parser.exit()


def cmd_docs(a) -> int:
    root = docs_root()
    if root is None:
        print('the docs are not in this install; reinstall Roborun: curl -fsSL https://roborun.app/install.sh | sh', file=sys.stderr)
        return 1
    if a.examples:
        target = Path(a.examples).expanduser().resolve()
        target.mkdir(parents=True, exist_ok=True)
        copied = []
        for item in sorted((root / 'examples').iterdir()):
            if item.suffix in ('.py', '.md') and not item.name.startswith('_'):
                shutil.copy2(item, target / item.name)
                copied.append(item.name)
        print(f'copied {len(copied)} files to {target}: {", ".join(copied)}')
        print(f'run one with Roborun\'s Python: roborun python {target / "robot_client.py"} status')
        return 0
    if a.name:
        print((root / DOCS[a.name]).read_text(), end='')
        return 0
    print('AI assistants: read this first: roborun docs agents  (one file: install, keys, up/down, profiles, the local API, '
          'every error and what to do)')
    for name, rel in DOCS.items():
        if name != 'examples':
            print(f'{name:11s} {root / rel}')
    print(f'{"examples":11s} {root / "examples"}  (roborun docs --examples DIR copies them)')
    print(f'{"local API":11s} {LOCAL_DOCS} and {LOCAL_DOCS.replace("/docs", "/openapi.json")} (while roborun up runs)')
    print(f'{"viewer":11s} http://127.0.0.1:8135/ (while roborun up runs: this robot\'s tasks, recordings, cameras and Stop)')
    return 0


def run_python(args, execv=None) -> int:
    """Roborun's own interpreter, where roborun_sdk is installed, with the arguments as given (`roborun python ...`)."""
    args = list(args)
    if args[:1] == ['--']:
        args = args[1:]
    (execv or os.execv)(sys.executable, [sys.executable, *args])
    return 0                                 # pragma: no cover - execv does not return


def cmd_python(a) -> int:
    return run_python(a.args)


def add_parsers(sub):
    d = sub.add_parser('docs', help='the guides (an AI assistant: roborun docs agents), the README and the examples; print one, or copy the examples',
                       description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    d.add_argument('name', nargs='?', choices=sorted(DOCS), help='print this document')
    d.add_argument('--examples', metavar='DIR', help='copy the examples into DIR')
    d.set_defaults(func=cmd_docs)
    p = sub.add_parser('python', help="run Python with Roborun's own interpreter (roborun_sdk importable): roborun python check.py",
                       add_help=False)
    p.add_argument('args', nargs=argparse.REMAINDER)
    p.set_defaults(func=cmd_python)
