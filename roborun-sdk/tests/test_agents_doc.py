"""docs/AGENTS.md, the one guide an AI assistant reads to run Roborun for someone, matches the code: every command and flag
it names exists, every local API route it names is served (the humanoid API here; the arm API from the runtime's source when
ROBORUN_RUNTIME_SRC points at it), every error code it explains is one the robot sends, and it ships, is printed and is pointed to."""
import contextlib
import io
import os
import re
from pathlib import Path

import pytest

from roborun_sdk import cli, devkit

SDK = Path(__file__).resolve().parents[1]
AGENTS = (SDK / 'docs' / 'AGENTS.md').read_text()
RUNTIME_API = Path(os.environ.get('ROBORUN_RUNTIME_SRC') or '/nonexistent') / 'api.py'   # the arm runtime's local API, when given
BODY_SERVER = (SDK / 'src' / 'roborun_sdk' / 'body' / 'server.py').read_text()


def _help(argv):
    out = io.StringIO()
    with contextlib.redirect_stdout(out), pytest.raises(SystemExit):
        cli.main(argv)
    return out.getvalue()


def _commands():
    """The subcommands `roborun --help-all` lists."""
    text = _help(['--help-all'])
    return set(re.findall(r'^    ([a-z][a-z-]+)\s', text, re.M))


def _spans():
    """Every `roborun ...` command written in the guide (inline code and code blocks)."""
    spans = re.findall(r'`(roborun [^`]+)`', AGENTS)
    return [s.replace('\\|', '|') for s in spans]


def test_it_is_short_enough_to_read_whole():
    assert len(AGENTS.encode()) < 16_000                      # about 4k tokens: it fits beside any task


def test_every_command_and_flag_it_names_exists():
    commands = _commands()
    assert {'up', 'down', 'status', 'stop', 'login', 'upgrade', 'uninstall', 'view', 'docs', 'profiles', 'setup'} <= commands
    seen = set()
    for span in _spans():
        tokens = span.split()[1:]
        if not tokens or tokens[0] in ('--version',):
            continue
        sub = tokens[0]
        assert sub in commands, span
        seen.add(sub)
        flags = [t.split('=')[0] for t in tokens[1:] if t.startswith('--')]
        if flags:
            words = [sub] + [t for t in tokens[1:tokens.index(flags[0]) if flags[0] in tokens else 1] if re.fullmatch(r'[a-z][a-z-]*', t)]
            text = _help([sub, '--help']) + (_help(words + ['--help']) if len(words) > 1 else '')
            for flag in flags:
                assert flag in text, (span, flag)
    assert {'up', 'login', 'uninstall', 'upgrade', 'status', 'stop', 'down', 'view', 'docs'} <= seen


def test_the_profiles_and_scenes_it_lists_exist():
    from roborun_sdk.arm_profiles import parse, runtime_profile
    from roborun_sdk.body import profiles as bodies
    from roborun_sdk.robot import bundled
    for name in ('yam', 'yam_dual', 'yam_metal'):                      # the names people type; the runtime's ids behind them
        rid = runtime_profile(name)
        assert (rid in bundled() or parse(rid) is not None) and f'`{name}`' in AGENTS
    assert 'yam_single' not in AGENTS and 'makerarm' not in AGENTS     # renamed: never taught
    for name in ('booster_k1', 'galaxea_r1'):
        assert bodies.is_body(name) and name in AGENTS
    for scene in bodies.SCENES:
        assert f'`{scene}`' in AGENTS
    sim = (SDK / 'src' / 'roborun_sdk' / 'mujoco_sim.py').read_text()
    for scene in ('cup', 'pickup-bench', 'mixed-objects', 'cluttered-bench', 'low-light'):
        assert f'`{scene}`' in AGENTS and (scene == 'cup' or scene in sim)


def _routes_named():
    out = set()
    for method, path in re.findall(r'\b(GET|POST) (/v1/[A-Za-z0-9_{}./-]+)', AGENTS):
        out.add((method, path.rstrip('.')))
    return out


def _pattern(path):
    return re.sub(r'\\\{[a-z_]+\\\}', '[^/]+', re.escape(path))


def _matches(path, served):
    """A documented route (maybe with {id}) is served: the same template, or one matches the other."""
    plain = re.sub(r'\{[a-z_]+\}', '{}', path)
    return any(re.sub(r'\{[a-z_]+\}', '{}', s) == plain or re.fullmatch(_pattern(path), s) or re.fullmatch(_pattern(s), path)
               for s in served)


def _body_routes():
    served = set(re.findall(r"'(/v1/[a-z0-9_/.-]+)'", BODY_SERVER))
    served |= {'/v1/body/moves/mv-0123456789ab', '/v1/tasks/mv-0123456789ab'}         # the move routes are a regex
    return served


def _arm_routes():
    text = RUNTIME_API.read_text()
    served = set(re.findall(r"'(/v1/[a-z0-9_/.-]+)'", text))
    served |= {'/v1' + p for p in re.findall(r"'(/[a-z{}_/.-]+)': \{'(?:get|post)'", text)}
    return served


def test_every_route_it_names_is_served():
    named = _routes_named()
    assert ('POST', '/v1/stop') in named and ('POST', '/v1/body/move') in named and ('POST', '/v1/tasks') in named
    body = _body_routes()
    arm = _arm_routes() if RUNTIME_API.is_file() else None
    for method, path in named:
        if _matches(path, body):
            continue
        if arm is None:
            pytest.skip('ROBORUN_RUNTIME_SRC is not set')
        assert _matches(path, arm), (method, path)


def test_every_error_code_it_explains_is_one_the_robot_sends():
    table = re.findall(r'^\| `([a-z_]+)`(?:, `([a-z_]+)`)?', AGENTS.split('## 7.')[1].split('## 8.')[0], re.M)
    codes = {c for row in table for c in row if c}
    sent = set(re.findall(r"code='([a-z_]+)'", BODY_SERVER)) | {'not_running'}
    client = (SDK / 'src' / 'roborun_sdk' / 'client.py').read_text()
    assert "'not_running'" in client
    if RUNTIME_API.is_file():
        described = re.search(r"'code': \{'type': 'string', 'description': '([^}]+)'\}", RUNTIME_API.read_text(), re.S).group(1)
        sent |= set(re.findall(r'[a-z_]+', described.replace("'", '')))
        assert {'invalid', 'not_found', 'busy', 'forbidden', 'method_not_allowed', 'unavailable', 'blur_unavailable',
                'stop_not_confirmed', 'unreachable', 'internal'} <= codes
    assert codes - sent <= ({'invalid', 'not_found', 'busy', 'forbidden', 'method_not_allowed', 'unavailable',
                             'blur_unavailable', 'internal'} if not RUNTIME_API.is_file() else set()), codes - sent
    for code in ('network_offline', 'service_unavailable', 'rate_limited', 'key_revoked', 'sdk_upgrade_required', 'signing_key_unknown'):
        assert code in AGENTS and code in (SDK / 'docs' / 'BUILDING.md').read_text()


def test_the_client_methods_it_lists_exist():
    from roborun_sdk.client import Robot
    listed = re.search(r'Methods: (.+?)\n\n', AGENTS.split('## 6.')[1], re.S).group(1)
    for name in re.findall(r'`([a-z_]+)\(', listed):
        assert callable(getattr(Robot, name, None)), name
    removed = {'clip'}                                       # a stub that only raises RobotError code 'removed': not offered
    assert 'clip' not in re.findall(r'`([a-z_]+)\(', listed)
    public = {n for n in dir(Robot) if not n.startswith('_') and callable(getattr(Robot, n))} - removed
    assert public <= set(re.findall(r'`([a-z_]+)\(', listed)), public - set(re.findall(r'`([a-z_]+)\(', listed))


def test_it_is_printed_first_and_pointed_to(capsys):
    assert cli.main(['docs', 'agents']) == 0
    assert capsys.readouterr().out.startswith('# Roborun: the guide for an AI assistant')
    assert cli.main(['docs']) == 0
    out = capsys.readouterr().out
    assert out.splitlines()[0].startswith('AI assistants: read this first: roborun docs agents') and 'AGENTS.md' in out
    assert 'roborun docs agents' in _help(['--help'])
    install = (SDK / 'install.sh').read_text()
    assert 'roborun docs agents' in install and 'AGENTS.md' in install
    assert 'roborun docs agents' in (SDK / 'README.md').read_text()
    assert devkit.DOCS['agents'] == 'docs/AGENTS.md'
    from roborun_sdk.body import server
    assert 'roborun docs agents' in server.DOCS_HTML and 'http://127.0.0.1:8135/' in server.DOCS_HTML      # the humanoid /v1/docs
    toml = (SDK / 'pyproject.toml').read_text()
    assert '"_docs/docs/*.md"' in toml                       # AGENTS.md ships in the wheel beside BUILDING.md
    for pattern in ('"viewer/*.html"', '"viewer/*.js"', '"viewer/*.css"'):
        assert pattern in toml                               # and so does the viewer
