"""Two Roborun installs on one computer account: every systemd unit, socket and folder a Roborun home uses is its own.

The default home (~/.config/roborun of the account) keeps today's unit names, so an upgraded robot keeps its running units.
Any other home (ROBORUN_HOME, or a different $HOME on the same account: systemd --user is still the same one) gets
roborun-<part>-<8 hex>, and its up / down / status / uninstall never start, stop or ask about the default home's units.

systemctl, systemd-run and loginctl are stubs on PATH here that record their arguments: nothing reaches the real systemd."""
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from roborun_sdk import robot

SRC = Path(__file__).resolve().parents[1] / 'src'
DEFAULT_UNITS = set(robot.UNITS)                    # roborun-local, roborun-robot, roborun-sim, roborun-station, roborun-cameras
STUB = '''#!/bin/sh
echo "{name} $*" >> "{log}"
case "$*" in
  *is-system-running*) echo running ;;
  *is-active*) echo inactive ;;
esac
exit 0
'''


@pytest.fixture
def systemd(tmp_path, monkeypatch):
    """A recording systemd --user: is-system-running says running, every unit is inactive, start and stop succeed."""
    stubs = tmp_path / 'stubs'
    stubs.mkdir()
    log = tmp_path / 'systemd-calls.log'
    log.write_text('')
    for name in ('systemctl', 'systemd-run', 'loginctl', 'journalctl'):
        path = stubs / name
        path.write_text(STUB.format(name=name, log=log))
        path.chmod(0o755)
    monkeypatch.setenv('PATH', f"{stubs}{os.pathsep}{os.environ.get('PATH', '')}")
    for name in ('systemctl', 'systemd-run', 'loginctl'):
        assert shutil.which(name) == str(stubs / name)          # never the real one
    monkeypatch.setenv('XDG_RUNTIME_DIR', str(tmp_path / 'run'))
    (tmp_path / 'run').mkdir()
    monkeypatch.delenv('MANIFEST_CAMERA_SOCKET', raising=False)
    monkeypatch.delenv('DBUS_SESSION_BUS_ADDRESS', raising=False)

    def calls():
        return [line.split() for line in log.read_text().splitlines()]
    return calls


def _home(tmp_path, monkeypatch, name, default=False):
    home = tmp_path / name
    home.mkdir(parents=True, exist_ok=True)
    monkeypatch.setattr(robot, 'HOME', home)
    monkeypatch.setattr(robot, 'CONFIG', home / 'robot.json')
    if default:
        monkeypatch.setattr(robot, 'account_home', lambda: home)
    else:
        monkeypatch.setattr(robot, 'account_home', lambda: tmp_path / 'account' / '.config' / 'roborun')
    return home


def _named_units(calls):
    """Every unit name a recorded systemctl / systemd-run call names."""
    names = []
    for argv in calls:
        if argv[0] == 'systemctl' and len(argv) > 3 and argv[2] in ('stop', 'start', 'restart', 'reset-failed', 'is-active', 'kill'):
            names.append(argv[3])
        if argv[0] == 'systemd-run':
            names += [a.split('=', 1)[1] for a in argv if a.startswith('--unit=')]
    return names


# ------------------------------------------------------------------------------------------------ the names
def test_the_default_home_keeps_today_s_names(tmp_path, monkeypatch, systemd):
    _home(tmp_path, monkeypatch, 'default', default=True)
    assert robot.install_suffix() == ''
    assert [robot.unit_name(u) for u in robot.UNITS] == list(robot.UNITS)
    assert robot.camera_socket() == str(tmp_path / 'run' / 'manifest' / 'camera.sock')
    for u in robot.UNITS:
        robot._stop(u)
    robot._unit('roborun-robot', ['true'], {})
    assert set(_named_units(systemd())) == DEFAULT_UNITS        # an upgraded install stops and starts the units it already runs


def test_another_home_gets_stable_names_of_its_own(tmp_path, monkeypatch):
    a = _home(tmp_path, monkeypatch, 'a')
    first = robot.unit_name('roborun-station')
    assert re.fullmatch(r'roborun-station-[0-9a-f]{8}', first)
    assert robot.unit_name('roborun-station') == first and robot.unit_name('roborun-station', a) == first      # stable
    assert robot.unit_name('roborun-station', tmp_path / 'b') != first                                        # distinct
    assert robot.unit_name('roborun-station', tmp_path / 'b' / '..' / 'a') == first                           # same folder, same name
    assert all(robot.unit_name(u) not in DEFAULT_UNITS for u in robot.UNITS)


def test_the_names_are_the_same_in_every_process_and_a_changed_home_env_is_not_the_default(tmp_path):
    """The units `roborun up` starts get ROBORUN_HOME; `roborun down` in another shell must name the same units. A sandbox
    that only changes $HOME is still this account's systemd, so it is not the default install either."""
    def name(env):
        env = dict({k: v for k, v in os.environ.items() if k != 'ROBORUN_HOME'}, PYTHONPATH=str(SRC), **env)
        return subprocess.run([sys.executable, '-c', 'from roborun_sdk import robot; print(robot.unit_name("roborun-robot"))'],
                              env=env, capture_output=True, text=True, check=True).stdout.strip()
    other = str(tmp_path / 'other')
    assert name(dict(ROBORUN_HOME=other)) == name(dict(ROBORUN_HOME=other)) != 'roborun-robot'
    sandbox = name(dict(HOME=str(tmp_path / 'sandbox-home')))
    assert re.fullmatch(r'roborun-robot-[0-9a-f]{8}', sandbox)


def test_each_home_has_its_own_camera_socket(tmp_path, monkeypatch, systemd):
    _home(tmp_path, monkeypatch, 'a')
    first = robot.camera_socket()
    _home(tmp_path, monkeypatch, 'b')
    second = robot.camera_socket()
    default = str(tmp_path / 'run' / 'manifest' / 'camera.sock')
    assert len({first, second, default}) == 3
    assert Path(first).parent.parent == Path(default).parent and Path(first).name == 'camera.sock'
    from roborun_sdk import cameras
    assert cameras.helper_socket(first) != cameras.helper_socket(default)       # the capture helper's socket too


# ------------------------------------------------------------------------------------------------ a second home never touches the first
def _never_the_default(calls):
    named = _named_units(calls)
    assert named, 'nothing was named: the stub was not used'
    assert not DEFAULT_UNITS & set(named), named
    assert not any(n.endswith('.service') and n[:-8] in DEFAULT_UNITS for n in named)
    suffix = robot.install_suffix()
    assert all(n.endswith(suffix) for n in named), named


def test_up_down_and_status_of_a_second_home_name_only_its_own_units(tmp_path, monkeypatch, systemd, capsys):
    from roborun_sdk.arms import settings
    from roborun_sdk.body import up as body_up
    home = _home(tmp_path, monkeypatch, 'second')
    monkeypatch.setenv('ROBORUN_HOME', str(home))
    monkeypatch.setattr(settings, 'settle_token', lambda *a: None)
    monkeypatch.setattr(robot, '_backend_up', lambda url, wait_s=0: True)
    monkeypatch.setattr(robot, '_port_taken', lambda port: False)
    monkeypatch.setattr(robot, '_busy', lambda url: False)
    monkeypatch.setattr(robot, 'gl_env', lambda: {})
    monkeypatch.setattr(robot, 'ready', lambda c, say=print: None)
    monkeypatch.setattr(robot, 'token_state', lambda server, key: ('offline', 'not reachable'))
    (home / 'token').write_text('rr_test_key\n')
    args = SimpleNamespace(profile='booster_k1', rig=None, left=None, right=None, sim='room', no_sim=False, robot='second',
                           key_file=str(home / 'token'), server='http://127.0.0.1:9', backend='http://127.0.0.1:8220',
                           body_agent=None, body_agent_key_file=None)
    assert body_up.up(args) == 0                                # roborun up: stops this home's parts, starts its robot and page
    started = [a for a in systemd() if a[0] == 'systemd-run']
    assert {a.split('=', 1)[1] for s in started for a in s if a.startswith('--unit=')} == {
        robot.unit_name('roborun-robot'), robot.unit_name('roborun-local')}
    assert robot.cmd_status(None) == 0                          # roborun status
    assert f"install           {home}" in capsys.readouterr().out
    assert robot.cmd_down(SimpleNamespace(park=False, release=False)) == 0      # roborun down
    for u in robot.UNITS:                                       # every part, directly
        robot._state(u); robot._stop(u)
    robot._unit('roborun-station', ['true'], {})
    _never_the_default(systemd())


def test_a_second_home_s_up_waits_for_a_taken_port_and_stops_nothing(tmp_path, monkeypatch, systemd):
    """The default install holds 8120 (and the station's 8091): the second one says so and how to move, and stops nothing."""
    home = _home(tmp_path, monkeypatch, 'second')
    robot.CONFIG.write_text('{}')
    monkeypatch.setattr(robot, '_port_taken', lambda port: True)
    with pytest.raises(SystemExit) as e:
        robot._check_ports(dict(backend='http://127.0.0.1:8120', live_port=0), None)
    text = str(e.value)
    assert 'port 8120' in text and 'another Roborun install' in text and 'roborun up --backend' in text and str(robot.CONFIG) in text
    with pytest.raises(SystemExit) as e:
        robot._check_ports(dict(backend='http://127.0.0.1:8220', sim='cup', live_port=8235), None)
    assert 'port 8220' in str(e.value)
    monkeypatch.setattr(robot, '_port_taken', lambda port: port == 8091)
    with pytest.raises(SystemExit) as e:
        robot._check_ports(dict(backend='http://127.0.0.1:8220', sim='cup', live_port=8235), None)
    assert 'port 8091' in str(e.value) and 'one per computer' in str(e.value)
    robot._check_ports(dict(backend='http://127.0.0.1:8220', sim='room', body=dict(profile='booster_k1'), live_port=8235), None)
    assert not any(a[0] == 'systemd-run' or (a[0] == 'systemctl' and a[2] == 'stop') for a in systemd())


def test_a_second_home_keeps_its_runtime_and_datasets_inside_it(tmp_path, monkeypatch):
    user = tmp_path / 'user'
    monkeypatch.setattr(robot.Path, 'home', classmethod(lambda cls: user))
    monkeypatch.delenv('MANIFEST_DREAMAPI', raising=False)
    monkeypatch.setattr(robot, 'HOME', user / '.config' / 'roborun')
    monkeypatch.delenv('ROBORUN_HOME', raising=False)
    assert robot._manifest_root() == str(user / 'roborun' / 'robot' / 'runtime')                # the default: as always
    assert robot.dataset_root('r') == user / 'roborun' / 'datasets' / 'r'
    other = tmp_path / 'other'
    monkeypatch.setattr(robot, 'HOME', other)
    monkeypatch.setenv('ROBORUN_HOME', str(other))
    assert robot._manifest_root() == str(other / 'roborun' / 'robot' / 'runtime')
    assert robot.dataset_root('r') == other / 'roborun' / 'datasets' / 'r'


def test_uninstall_of_a_second_home_removes_only_that_home(tmp_path, monkeypatch, systemd, capsys):
    user = tmp_path / 'user'
    default_root = user / '.roborun'                                    # the default install, running from ~/.roborun
    (default_root / 'python' / 'bin').mkdir(parents=True); (default_root / 'tools').mkdir()
    (default_root / 'python' / 'bin' / 'roborun').write_text('#!/bin/sh\n')
    (user / '.local' / 'bin').mkdir(parents=True)
    (user / '.local' / 'bin' / 'roborun').symlink_to(default_root / 'python' / 'bin' / 'roborun')
    (user / '.bashrc').write_text('alias ll="ls -l"\n' + robot.RC_BLOCK)
    (user / '.config' / 'roborun').mkdir(parents=True)
    (user / '.config' / 'roborun' / 'robot.json').write_text('{}')
    (user / '.config' / 'roborun' / robot.LINGER_MARK).write_text('alice\n')
    runtime = user / 'roborun' / 'robot' / 'runtime'
    runtime.mkdir(parents=True); (runtime / 'RUNTIME.json').write_text('{"version": "x"}')
    monkeypatch.setattr(robot.Path, 'home', classmethod(lambda cls: user))
    home = _home(tmp_path, monkeypatch, 'second')
    monkeypatch.setenv('ROBORUN_HOME', str(home))
    (home / robot.LINGER_MARK).write_text('alice\n')                   # this home turned linger on too
    robot.CONFIG.write_text(json.dumps(dict(manifest=str(runtime))))   # even pointed at the default's runtime
    monkeypatch.setattr(sys.stdin, 'isatty', lambda: False)
    assert robot.cmd_uninstall(SimpleNamespace(yes=True, keep_data=False, purge=True, park=False, release=False)) == 0
    out = capsys.readouterr().out
    assert not home.exists()                                            # its own home: gone
    assert (default_root / 'tools').is_dir() and (user / '.local' / 'bin' / 'roborun').is_symlink()
    assert robot.RC_BLOCK in (user / '.bashrc').read_text() and (runtime / 'RUNTIME.json').is_file()
    assert (user / '.config' / 'roborun' / 'robot.json').is_file()
    assert not any(a[:2] == ['loginctl', 'disable-linger'] for a in systemd())     # the default install still needs linger
    assert 'left alone' in out
    _never_the_default(systemd())


@pytest.mark.parametrize('account_via', ['pwd', 'home'])
def test_uninstall_of_a_second_home_run_by_the_installed_command_keeps_the_default_install(tmp_path, monkeypatch, systemd,
                                                                                           capsys, account_via):
    """ROBORUN_HOME=x roborun uninstall, with the usual roborun command: that Python is ~/.roborun/python, which is the
    default install's. Neither ~/.roborun nor ~/.local/bin/roborun may go."""
    user = tmp_path / 'user'
    default_root = user / '.roborun'
    (default_root / 'python' / 'bin').mkdir(parents=True); (default_root / 'tools').mkdir()
    (default_root / 'python' / 'bin' / 'roborun').write_text('#!/bin/sh\n')
    (user / '.local' / 'bin').mkdir(parents=True)
    (user / '.local' / 'bin' / 'roborun').symlink_to(default_root / 'python' / 'bin' / 'roborun')
    (user / '.bashrc').write_text(robot.RC_BLOCK)
    home = _home(tmp_path, monkeypatch, 'second')
    if account_via == 'pwd':                    # $HOME changed, the account's own home is still user
        monkeypatch.setattr(robot.Path, 'home', classmethod(lambda cls: tmp_path / 'elsewhere'))
        monkeypatch.setattr(robot, 'account_home', lambda: user / '.config' / 'roborun')
    else:
        monkeypatch.setattr(robot.Path, 'home', classmethod(lambda cls: user))
    monkeypatch.setattr(sys, 'prefix', str(default_root / 'python'))       # the installed roborun
    assert robot.install_root() == default_root
    monkeypatch.setenv('ROBORUN_HOME', str(home))
    monkeypatch.setattr(sys.stdin, 'isatty', lambda: False)
    assert robot.cmd_uninstall(SimpleNamespace(yes=True, keep_data=False, purge=True, park=False, release=False)) == 0
    out = capsys.readouterr().out
    assert not home.exists()
    assert (default_root / 'tools').is_dir() and (default_root / 'python' / 'bin' / 'roborun').is_file()
    assert (user / '.local' / 'bin' / 'roborun').is_symlink()
    assert robot.RC_BLOCK in (user / '.bashrc').read_text()
    assert f'removed {default_root}' not in out and 'left alone' in out
    _never_the_default(systemd())


def _account(tmp_path, monkeypatch):
    """The account's default install, set up the way install.sh and roborun up leave it: ~/.roborun with its Python, the
    roborun command, the PATH line, its settings (with linger on) and its runtime. ~ is the account's own folder (pw_dir)."""
    user = tmp_path / 'user'
    root = user / '.roborun'
    (root / 'python' / 'bin').mkdir(parents=True); (root / 'tools').mkdir()
    (root / 'python' / 'bin' / 'roborun').write_text('#!/bin/sh\n')
    (user / '.local' / 'bin').mkdir(parents=True)
    (user / '.local' / 'bin' / 'roborun').symlink_to(root / 'python' / 'bin' / 'roborun')
    (user / '.bashrc').write_text('alias ll="ls -l"\n' + robot.RC_BLOCK)
    (user / '.config' / 'roborun').mkdir(parents=True)
    (user / '.config' / 'roborun' / 'robot.json').write_text('{}')
    (user / '.config' / 'roborun' / robot.LINGER_MARK).write_text('alice\n')
    runtime = user / 'roborun' / 'robot' / 'runtime'
    runtime.mkdir(parents=True); (runtime / 'RUNTIME.json').write_text('{"version": "x"}')
    monkeypatch.setattr(robot, 'account_home', lambda: user / '.config' / 'roborun')
    monkeypatch.setattr(sys.stdin, 'isatty', lambda: False)
    return user


def _default_install_untouched(user):
    root = user / '.roborun'
    assert (root / 'tools').is_dir() and (root / 'python' / 'bin' / 'roborun').is_file()
    assert (user / '.local' / 'bin' / 'roborun').is_symlink()
    assert robot.RC_BLOCK in (user / '.bashrc').read_text()
    assert (user / '.config' / 'roborun' / 'robot.json').is_file()
    assert (user / '.config' / 'roborun' / robot.LINGER_MARK).is_file()
    assert (user / 'roborun' / 'robot' / 'runtime' / 'RUNTIME.json').is_file()


def _sandbox_home(tmp_path, monkeypatch, roborun_home):
    """A sandbox that changes $HOME (as the test and verify scripts do), and ROBORUN_HOME: unset, the sandbox's default
    folder, or a folder of its own (both set)."""
    sandbox = tmp_path / 'sandbox'
    sandbox.mkdir()
    monkeypatch.setattr(robot.Path, 'home', classmethod(lambda cls: sandbox))
    home = {'unset': sandbox / '.config' / 'roborun', 'sandbox-default': sandbox / '.config' / 'roborun',
            'own': tmp_path / 'second'}[roborun_home]
    home.mkdir(parents=True)
    monkeypatch.setattr(robot, 'HOME', home)
    monkeypatch.setattr(robot, 'CONFIG', home / 'robot.json')
    if roborun_home == 'unset':
        monkeypatch.delenv('ROBORUN_HOME', raising=False)
    else:
        monkeypatch.setenv('ROBORUN_HOME', str(home))
    return sandbox, home


@pytest.mark.parametrize('roborun_home', ['unset', 'sandbox-default', 'own'])
def test_uninstall_in_a_sandbox_that_changes_home_never_removes_the_account_s_install(tmp_path, monkeypatch, systemd, capsys,
                                                                                    roborun_home):
    """HOME=/tmp/sandbox roborun uninstall --purge, run by the account's installed roborun (~/.roborun/python): its units are
    the sandbox's (suffixed), so the sandbox is a second install, and the account's ~/.roborun (the software the live robot's
    units run from), the roborun command, the PATH line, ~/roborun and the linger setting must all stay."""
    user = _account(tmp_path, monkeypatch)
    sandbox, home = _sandbox_home(tmp_path, monkeypatch, roborun_home)
    monkeypatch.setattr(sys, 'prefix', str(user / '.roborun' / 'python'))
    assert robot.install_root() == user / '.roborun' and robot.second_install() and robot.install_suffix()
    (home / robot.LINGER_MARK).write_text('alice\n')
    assert robot.cmd_uninstall(SimpleNamespace(yes=True, keep_data=False, purge=True, park=False, release=False)) == 0
    out = capsys.readouterr().out
    _default_install_untouched(user)
    assert not home.exists()                                            # the sandbox's own home: gone
    assert f'removed {user / ".roborun"}' not in out and 'left alone' in out
    assert not any(a[:2] == ['loginctl', 'disable-linger'] for a in systemd())
    _never_the_default(systemd())


def test_uninstall_of_a_sandbox_s_own_install_removes_it_and_leaves_the_account_s(tmp_path, monkeypatch, systemd, capsys):
    """install.sh run with HOME=/tmp/sandbox put Roborun in the sandbox: that roborun's uninstall removes the sandbox's
    ~/.roborun, command and PATH line (they are its own), and nothing of the account's."""
    user = _account(tmp_path, monkeypatch)
    sandbox, home = _sandbox_home(tmp_path, monkeypatch, 'unset')
    root = sandbox / '.roborun'
    (root / 'python' / 'bin').mkdir(parents=True); (root / 'tools').mkdir()
    (root / 'python' / 'bin' / 'roborun').write_text('#!/bin/sh\n')
    (sandbox / '.local' / 'bin').mkdir(parents=True)
    (sandbox / '.local' / 'bin' / 'roborun').symlink_to(root / 'python' / 'bin' / 'roborun')
    (sandbox / '.bashrc').write_text(robot.RC_BLOCK)
    runtime = sandbox / 'roborun' / 'robot' / 'runtime'
    runtime.mkdir(parents=True); (runtime / 'RUNTIME.json').write_text('{}')
    monkeypatch.setattr(sys, 'prefix', str(root / 'python'))
    assert robot.cmd_uninstall(SimpleNamespace(yes=True, keep_data=False, purge=True, park=False, release=False)) == 0
    assert not root.exists() and not (sandbox / '.local' / 'bin' / 'roborun').is_symlink()
    assert robot.RC_BLOCK not in (sandbox / '.bashrc').read_text()
    assert not home.exists() and not (sandbox / 'roborun').exists()
    _default_install_untouched(user)
    _never_the_default(systemd())


@pytest.mark.parametrize('where', ['account', 'parent', 'inside'])
def test_a_second_home_that_holds_the_default_install_is_never_deleted(tmp_path, monkeypatch, systemd, capsys, where):
    """ROBORUN_HOME=~ (or /home) roborun uninstall --purge: deleting it would delete the default install, so it is left
    alone. A folder inside ~/.roborun: its pid files and station files are inside the default install, so uninstall stops
    before it asks or stops anything, with nothing changed."""
    user = _account(tmp_path, monkeypatch)
    monkeypatch.setattr(robot.Path, 'home', classmethod(lambda cls: user))
    home = {'account': user, 'parent': tmp_path, 'inside': user / '.roborun' / 'x'}[where]
    home.mkdir(exist_ok=True)
    monkeypatch.setattr(robot, 'HOME', home)
    monkeypatch.setattr(robot, 'CONFIG', home / 'robot.json')
    monkeypatch.setenv('ROBORUN_HOME', str(home))
    monkeypatch.setattr(sys, 'prefix', str(user / '.roborun' / 'python'))
    assert robot.second_install() and robot.touches_default_install(home)
    if where == 'inside':
        with pytest.raises(SystemExit, match='roborun uninstall stopped.*Nothing was changed'):
            robot.cmd_uninstall(SimpleNamespace(yes=True, keep_data=False, purge=True, park=False, release=False))
    else:
        assert robot.cmd_uninstall(SimpleNamespace(yes=True, keep_data=False, purge=True, park=False, release=False)) == 0
    assert home.is_dir()
    _default_install_untouched(user)


def _sandbox_install(sandbox):
    """install.sh run with HOME=sandbox: the sandbox's own ~/.roborun, command, PATH line, settings and runtime."""
    root = sandbox / '.roborun'
    (root / 'python' / 'bin').mkdir(parents=True); (root / 'tools').mkdir()
    (root / 'python' / 'bin' / 'roborun').write_text('#!/bin/sh\n')
    (sandbox / '.local' / 'bin').mkdir(parents=True)
    (sandbox / '.local' / 'bin' / 'roborun').symlink_to(root / 'python' / 'bin' / 'roborun')
    (sandbox / '.bashrc').write_text(robot.RC_BLOCK)
    (sandbox / '.config' / 'roborun').mkdir(parents=True, exist_ok=True)
    (sandbox / '.config' / 'roborun' / 'robot.json').write_text('{}')
    runtime = sandbox / 'roborun' / 'robot' / 'runtime'
    runtime.mkdir(parents=True); (runtime / 'RUNTIME.json').write_text('{}')
    return root


def _sandbox_install_untouched(sandbox):
    root = sandbox / '.roborun'
    assert (root / 'tools').is_dir() and (root / 'python' / 'bin' / 'roborun').is_file()
    assert (sandbox / '.local' / 'bin' / 'roborun').is_symlink()
    assert robot.RC_BLOCK in (sandbox / '.bashrc').read_text()
    assert (sandbox / '.config' / 'roborun' / 'robot.json').is_file()
    assert (sandbox / 'roborun' / 'robot' / 'runtime' / 'RUNTIME.json').is_file()


@pytest.mark.parametrize('runs_from', ['sandbox', 'account'])
def test_uninstall_of_a_third_home_keeps_the_sandbox_s_and_the_account_s_installs(tmp_path, monkeypatch, systemd, capsys,
                                                                                 runs_from):
    """HOME=/tmp/sandbox (with its own install.sh install) and ROBORUN_HOME=/tmp/second roborun uninstall --purge: the
    sandbox's ~/.roborun, command, PATH line, settings and runtime are the sandbox's default home's, which its own units still
    use; only /tmp/second goes. The account's install stays too."""
    user = _account(tmp_path, monkeypatch)
    sandbox, home = _sandbox_home(tmp_path, monkeypatch, 'own')
    root = _sandbox_install(sandbox)
    monkeypatch.setattr(sys, 'prefix', str((root if runs_from == 'sandbox' else user / '.roborun') / 'python'))
    assert robot.second_install() and not robot.home_is_ours()
    assert robot.protected(root) and robot.protected(sandbox / '.config' / 'roborun') and not robot.protected(home)
    assert robot.cmd_uninstall(SimpleNamespace(yes=True, keep_data=False, purge=True, park=False, release=False)) == 0
    out = capsys.readouterr().out
    assert not home.exists()
    _sandbox_install_untouched(sandbox)
    _default_install_untouched(user)
    assert 'removed ' + str(root) not in out and 'left alone' in out
    _never_the_default(systemd())


@pytest.mark.parametrize('roborun_home', ['unset', 'sandbox-default'])
def test_the_verifier_s_home_only_sandbox_keeps_the_account_s_roborun(tmp_path, monkeypatch, systemd, capsys, roborun_home):
    """The repro: HOME=/tmp/sandbox, ROBORUN_HOME unset (or $HOME/.config/roborun), the account's installed roborun,
    uninstall --yes --keep-data. ~/.roborun of the account stays, and its units are never named."""
    user = _account(tmp_path, monkeypatch)
    sandbox, home = _sandbox_home(tmp_path, monkeypatch, roborun_home)
    monkeypatch.setattr(sys, 'prefix', str(user / '.roborun' / 'python'))
    assert robot.home_is_ours() and robot.protected(user / '.roborun') and not robot.protected(sandbox / '.roborun')
    assert robot.cmd_uninstall(SimpleNamespace(yes=True, keep_data=True, purge=False, park=False, release=False)) == 0
    out = capsys.readouterr().out
    _default_install_untouched(user)
    assert home.is_dir()                                                # --keep-data
    assert f'removed {user / ".roborun"}' not in out
    _never_the_default(systemd())


@pytest.mark.parametrize('roborun_home', ['unset', 'own'])
def test_a_second_install_removes_only_software_inside_its_own_homes(tmp_path, monkeypatch, systemd, capsys, roborun_home):
    """The password database names a different folder than the one the running roborun was installed in (a user namespace,
    a moved account, a container): that ~/.roborun is not recognised as the default install's, yet a second install still
    removes only an install folder inside its own home ($HOME of a sandbox, or ROBORUN_HOME)."""
    user = _account(tmp_path, monkeypatch)
    monkeypatch.setattr(robot, 'account_home', lambda: tmp_path / 'root' / '.config' / 'roborun')   # pw_dir: elsewhere
    sandbox, home = _sandbox_home(tmp_path, monkeypatch, roborun_home)
    monkeypatch.setattr(sys, 'prefix', str(user / '.roborun' / 'python'))
    assert robot.second_install() and not robot.protected(user / '.roborun')
    assert robot.cmd_uninstall(SimpleNamespace(yes=True, keep_data=True, purge=False, park=False, release=False)) == 0
    out = capsys.readouterr().out
    _default_install_untouched(user)
    assert f'removed {user / ".roborun"}' not in out and 'left alone' in out


def test_upgrade_of_a_third_home_never_replaces_the_sandbox_s_software(tmp_path, monkeypatch):
    """HOME=/tmp/sandbox with its own install, ROBORUN_HOME=/tmp/second: install.sh would go into the sandbox's .roborun,
    which the sandbox's own robot runs from. roborun upgrade refuses before downloading anything."""
    user = _account(tmp_path, monkeypatch)
    sandbox, home = _sandbox_home(tmp_path, monkeypatch, 'own')
    _sandbox_install(sandbox)
    ran = []
    monkeypatch.setattr(subprocess, 'run', lambda *a, **k: ran.append(a) or SimpleNamespace(returncode=1))
    monkeypatch.setattr(robot, 'latest_build', lambda timeout=15: ran.append('latest') or 'b2')
    with pytest.raises(SystemExit, match='second Roborun install.*Nothing was changed'):
        robot.cmd_upgrade(SimpleNamespace(force=True))
    assert not ran
    _sandbox_install_untouched(sandbox)


def test_the_default_install_is_never_protected(tmp_path, monkeypatch):
    user = _account(tmp_path, monkeypatch)
    monkeypatch.setattr(robot.Path, 'home', classmethod(lambda cls: user))
    monkeypatch.setattr(robot, 'HOME', user / '.config' / 'roborun')
    monkeypatch.delenv('ROBORUN_HOME', raising=False)
    assert not robot.second_install() and not robot.home_is_ours() and not robot.protected(user / '.roborun')


def test_what_counts_as_the_default_install_s(tmp_path, monkeypatch):
    user = tmp_path / 'user'
    monkeypatch.setattr(robot, 'account_home', lambda: user / '.config' / 'roborun')
    for p in (user, user / '.roborun', user / '.roborun' / 'python', user / 'roborun', user / '.config', user / '.config' / 'roborun',
              user / '.local' / 'bin' / 'roborun', user / '.local' / 'bin', tmp_path, Path('/')):   # .local/bin holds the command
        assert robot.touches_default_install(p), p
    for p in (user / 'second', user / '.config' / 'roborun-test', tmp_path / 'sandbox', tmp_path / 'sandbox' / '.roborun',
              user / '.local' / 'share', user / 'roborun-test'):
        assert not robot.touches_default_install(p), p
    (tmp_path / 'link').symlink_to(user)                    # a symlink to the account's folder is the account's folder
    assert robot.touches_default_install(tmp_path / 'link')


@pytest.mark.parametrize('roborun_home', ['own', 'sandbox'])
def test_upgrade_of_a_second_home_never_replaces_the_default_install_s_software(tmp_path, monkeypatch, roborun_home):
    """install.sh always installs into $HOME/.roborun: with ROBORUN_HOME set and the account's $HOME that is the default
    install's software, so roborun upgrade refuses before downloading anything. A sandbox's own $HOME upgrades its own."""
    user = _account(tmp_path, monkeypatch)
    ran = []
    monkeypatch.setattr(subprocess, 'run',
                        lambda *a, **k: ran.append(a) or SimpleNamespace(returncode=1))
    monkeypatch.setattr(robot, 'installed_build', lambda: 'b1')
    monkeypatch.setattr(robot, 'latest_build', lambda timeout=15: 'b1')
    if roborun_home == 'own':
        monkeypatch.setattr(robot.Path, 'home', classmethod(lambda cls: user))
        monkeypatch.setattr(robot, 'HOME', tmp_path / 'second')
        with pytest.raises(SystemExit, match='second Roborun install.*Nothing was changed'):
            robot.cmd_upgrade(SimpleNamespace(force=True))
    else:
        _sandbox_home(tmp_path, monkeypatch, 'unset')
        assert robot.cmd_upgrade(SimpleNamespace(force=False)) == 0        # past the guard: "already on the newest"
    assert not ran
    _default_install_untouched(user)


def test_the_default_home_is_not_a_second_install(tmp_path, monkeypatch):
    user = _account(tmp_path, monkeypatch)
    monkeypatch.setattr(robot, 'HOME', user / '.config' / 'roborun')
    assert not robot.second_install() and robot.install_suffix() == ''
    monkeypatch.setenv('ROBORUN_HOME', str(user / '.config' / 'roborun'))
    assert not robot.own_home()


@pytest.mark.parametrize('roborun_home', ['unset', 'sandbox-default', 'own'])
def test_the_runtime_folder_follows_the_same_rule_as_uninstall(tmp_path, monkeypatch, roborun_home):
    """A $HOME-only sandbox (ROBORUN_HOME unset, or set to $HOME/.config/roborun: the same home) keeps its runtime in its own
    $HOME/roborun, which its uninstall removes; ROBORUN_HOME elsewhere keeps it inside that home. Never the account's."""
    user = _account(tmp_path, monkeypatch)
    sandbox, home = _sandbox_home(tmp_path, monkeypatch, roborun_home)
    want = home / 'roborun' if roborun_home == 'own' else sandbox / 'roborun'
    assert robot.data_top() == want and not robot.protected(want) and not robot.touches_default_install(want)
    assert robot.own_home() == (roborun_home == 'own')


def test_uninstall_assumes_an_unreadable_settings_file_is_there(tmp_path, monkeypatch):
    """The account's settings in a folder this process may not read (a sandbox, another uid): keep linger, do not crash."""
    def denied(self, *a, **k):
        raise PermissionError(13, 'Permission denied', str(self))
    monkeypatch.setattr(robot.Path, 'exists', denied)
    assert robot._maybe_exists(tmp_path / 'robot.json') is True


def test_up_flags_move_a_set_up_install_to_its_own_ports(tmp_path, monkeypatch):
    _home(tmp_path, monkeypatch, 'second')
    c = dict(backend='http://127.0.0.1:8120')
    robot.CONFIG.write_text(json.dumps(c))
    a = SimpleNamespace(backend=None, live_port=None)
    robot.backend_arg(a)
    robot.remember_ports(a, c)                                  # nothing typed: nothing changes
    assert a.backend == 'http://127.0.0.1:8120' and json.loads(robot.CONFIG.read_text()) == dict(backend='http://127.0.0.1:8120')
    a = SimpleNamespace(backend='http://127.0.0.1:8220/', live_port=8235)
    robot.backend_arg(a)
    robot.remember_ports(a, c)
    assert json.loads(robot.CONFIG.read_text()) == dict(backend='http://127.0.0.1:8220', live_port=8235)
    for bad in ('http://10.0.0.2:8220', 'http://127.0.0.1', 'https://127.0.0.1:8220'):
        a = SimpleNamespace(backend=bad, live_port=None); robot.backend_arg(a)
        with pytest.raises(SystemExit, match='an address on this computer'):
            robot.remember_ports(a, dict(c))


# ------------------------------------------------------------------------------------------------ paths that resolve into the default install
def _no_network(monkeypatch):
    ran = []
    monkeypatch.setattr(subprocess, 'run', lambda *a, **k: ran.append(a) or SimpleNamespace(returncode=1))
    monkeypatch.setattr(robot, 'installed_build', lambda: 'b1')
    monkeypatch.setattr(robot, 'latest_build', lambda timeout=15: ran.append('latest') or 'b2')
    return ran


@pytest.mark.parametrize('linked', ['.roborun', '.local/bin', '.bashrc', 'roborun'])
def test_upgrade_in_a_sandbox_with_a_symlink_into_the_account_s_install_is_refused(tmp_path, monkeypatch, linked):
    """The verifier's repro: HOME=/tmp/sandbox, ROBORUN_HOME unset, and sandbox/.roborun a symlink to the account's
    ~/.roborun (or its .local/bin, startup file or roborun data folder). The sandbox's home is its own (home_is_ours), but
    install.sh would reinstall into the live robot's software: refused before anything is downloaded."""
    user = _account(tmp_path, monkeypatch)
    sandbox, home = _sandbox_home(tmp_path, monkeypatch, 'unset')
    target = user / linked
    (sandbox / linked).parent.mkdir(parents=True, exist_ok=True)
    (sandbox / linked).symlink_to(target)
    monkeypatch.setattr(sys, 'prefix', str(sandbox / '.roborun' / 'python'))
    assert robot.second_install() and robot.home_is_ours()
    ran = _no_network(monkeypatch)
    with pytest.raises(SystemExit, match=r'roborun upgrade stopped: this is a second Roborun install.*Nothing was changed'):
        robot.cmd_upgrade(SimpleNamespace(force=True))
    assert not ran
    _default_install_untouched(user)


def test_the_verifier_s_symlinked_roborun_repro_now_refuses_upgrade_and_uninstall_keeps_the_account(tmp_path, monkeypatch, systemd,
                                                                                                  capsys):
    user = _account(tmp_path, monkeypatch)
    sandbox, home = _sandbox_home(tmp_path, monkeypatch, 'unset')
    (sandbox / '.roborun').symlink_to(user / '.roborun')
    monkeypatch.setattr(sys, 'prefix', str(sandbox / '.roborun' / 'python'))      # the account's roborun, via the link
    with monkeypatch.context() as m:
        ran = _no_network(m)
        with pytest.raises(SystemExit, match='Nothing was changed'):
            robot.cmd_upgrade(SimpleNamespace(force=False))
    assert not ran
    assert robot.cmd_uninstall(SimpleNamespace(yes=True, keep_data=False, purge=True, park=False, release=False)) == 0
    _default_install_untouched(user)
    assert (sandbox / '.roborun').is_symlink()


@pytest.mark.parametrize('points_at', ['sandbox', 'sandbox-with-linked-roborun'])
def test_a_symlinked_home_counts_by_where_it_resolves(tmp_path, monkeypatch, points_at):
    """HOME=/tmp/link, a symlink to a sandbox folder: a second install that upgrades its own .roborun, unless what the link
    resolves to has a .roborun that is the account's."""
    user = _account(tmp_path, monkeypatch)
    real = tmp_path / 'real-sandbox'
    (real / '.config' / 'roborun').mkdir(parents=True)
    if points_at == 'sandbox-with-linked-roborun':
        (real / '.roborun').symlink_to(user / '.roborun')
    link = tmp_path / 'link'
    link.symlink_to(real)
    monkeypatch.setattr(robot.Path, 'home', classmethod(lambda cls: link))
    monkeypatch.setattr(robot, 'HOME', link / '.config' / 'roborun')
    monkeypatch.setattr(robot, 'CONFIG', link / '.config' / 'roborun' / 'robot.json')
    monkeypatch.delenv('ROBORUN_HOME', raising=False)
    assert robot.second_install() and robot.home_is_ours()
    ran = _no_network(monkeypatch)
    monkeypatch.setattr(robot, 'latest_build', lambda timeout=15: 'b1')            # already the newest: past the guard, no install
    if points_at == 'sandbox':
        assert robot.cmd_upgrade(SimpleNamespace(force=False)) == 0
    else:
        with pytest.raises(SystemExit, match='Nothing was changed'):
            robot.cmd_upgrade(SimpleNamespace(force=False))
    assert not ran
    _default_install_untouched(user)


def test_a_home_that_is_a_symlink_to_the_account_s_folder_is_the_default_install(tmp_path, monkeypatch, systemd):
    """HOME=/tmp/link -> the account's own folder: the same folders, so the same install as before this change (plain unit
    names, nothing refused)."""
    user = _account(tmp_path, monkeypatch)
    link = tmp_path / 'link'
    link.symlink_to(user)
    monkeypatch.setattr(robot.Path, 'home', classmethod(lambda cls: link))
    monkeypatch.setattr(robot, 'HOME', link / '.config' / 'roborun')
    monkeypatch.setattr(robot, 'CONFIG', link / '.config' / 'roborun' / 'robot.json')
    monkeypatch.delenv('ROBORUN_HOME', raising=False)
    assert not robot.second_install() and robot.install_suffix() == ''
    robot.refuse_default_install('upgrade', robot.installer_paths() + robot.home_paths())      # never for the default install
    robot.refuse_default_install('up', robot.home_paths())
    ran = _no_network(monkeypatch)
    monkeypatch.setattr(robot, 'latest_build', lambda timeout=15: 'b1')
    assert robot.cmd_upgrade(SimpleNamespace(force=False)) == 0
    assert not ran


def test_the_default_install_is_never_refused(tmp_path, monkeypatch):
    user = _account(tmp_path, monkeypatch)
    monkeypatch.setattr(robot.Path, 'home', classmethod(lambda cls: user))
    monkeypatch.setattr(robot, 'HOME', user / '.config' / 'roborun')
    monkeypatch.setattr(robot, 'CONFIG', user / '.config' / 'roborun' / 'robot.json')
    monkeypatch.delenv('ROBORUN_HOME', raising=False)
    for command in ('up', 'setup', 'down', 'upgrade', 'uninstall'):
        robot.refuse_default_install(command, robot.installer_paths() + robot.home_paths() + [user / '.roborun', user])


@pytest.mark.parametrize('inside', ['.config/roborun/x', '.roborun/x', 'roborun', '.', '.config'])
def test_up_and_setup_with_roborun_home_in_the_default_install_write_nothing(tmp_path, monkeypatch, systemd, inside):
    """ROBORUN_HOME=~/.config/roborun/x (or inside ~/.roborun, the account's ~/roborun, ~ itself, ~/.config): a second
    install whose settings, runtime or data folder would land in the default install's folders. up and setup stop before
    writing anything; upgrade stops too."""
    user = _account(tmp_path, monkeypatch)
    monkeypatch.setattr(robot.Path, 'home', classmethod(lambda cls: user))
    home = (user / inside).resolve()
    monkeypatch.setattr(robot, 'HOME', home)
    monkeypatch.setattr(robot, 'CONFIG', home / 'robot.json')
    monkeypatch.setenv('ROBORUN_HOME', str(home))
    monkeypatch.delenv('MANIFEST_DREAMAPI', raising=False)
    assert robot.second_install()
    before = sorted(str(p) for p in user.rglob('*'))
    a = SimpleNamespace(manifest=None, sim=None, profile=None, rig=None, robot='r', key_file=None, server=None, detect=False)
    for cmd in (robot.cmd_up, robot.cmd_setup):
        with pytest.raises(SystemExit, match='stopped: this is a second Roborun install.*Nothing was changed'):
            cmd(a)
    ran = _no_network(monkeypatch)
    with pytest.raises(SystemExit, match='Nothing was changed'):
        robot.cmd_upgrade(SimpleNamespace(force=True))
    assert not ran and not systemd()
    assert sorted(str(p) for p in user.rglob('*')) == before
    _default_install_untouched(user)


@pytest.mark.parametrize('how', ['runtime-flag', 'robot-json', 'data-link', 'env'])
def test_up_of_a_second_install_never_writes_the_default_install_s_runtime(tmp_path, monkeypatch, systemd, how):
    """A second install pointed at the account's runtime (--runtime, its robot.json, a symlinked data folder in a sandbox,
    MANIFEST_DREAMAPI): roborun up would update that runtime and its managed config under the live robot. Refused."""
    user = _account(tmp_path, monkeypatch)
    sandbox, home = _sandbox_home(tmp_path, monkeypatch, 'unset')
    live = user / 'roborun' / 'robot' / 'runtime'
    monkeypatch.delenv('MANIFEST_DREAMAPI', raising=False)
    a = SimpleNamespace(manifest=None, sim=None, profile=None, rig=None, robot='r', key_file=None, server=None, detect=False)
    if how == 'runtime-flag':
        a.manifest = str(live)
    elif how == 'robot-json':
        robot.CONFIG.write_text(json.dumps(dict(manifest=str(live))))
    elif how == 'data-link':
        (sandbox / 'roborun').symlink_to(user / 'roborun')
    else:
        monkeypatch.setenv('MANIFEST_DREAMAPI', str(live))
    with pytest.raises(SystemExit, match='roborun up stopped.*Nothing was changed'):
        robot.cmd_up(a)
    assert not systemd()
    _default_install_untouched(user)


def test_down_of_a_second_install_whose_pid_folder_is_the_default_s_stops_nothing(tmp_path, monkeypatch, systemd):
    """Without systemd, down kills the processes in <home>/run: a sandbox whose run folder is a symlink to the default
    home's would stop the live robot. Refused."""
    user = _account(tmp_path, monkeypatch)
    sandbox, home = _sandbox_home(tmp_path, monkeypatch, 'unset')
    (user / '.config' / 'roborun' / 'run').mkdir()
    (user / '.config' / 'roborun' / 'run' / 'roborun-robot.pid').write_text('1\n')
    (home / 'run').symlink_to(user / '.config' / 'roborun' / 'run')
    with pytest.raises(SystemExit, match='roborun down stopped.*Nothing was changed'):
        robot.cmd_down(SimpleNamespace(park=False, release=False))
    assert not systemd()
    assert (user / '.config' / 'roborun' / 'run' / 'roborun-robot.pid').is_file()


@pytest.mark.parametrize('roborun_home', ['unset', 'sandbox-default', 'own'])
def test_an_ordinary_second_install_passes_the_guard(tmp_path, monkeypatch, roborun_home):
    """A sandbox (or ROBORUN_HOME elsewhere) with folders of its own: up, setup, down and uninstall go on as before; upgrade
    too in a sandbox's own $HOME (ROBORUN_HOME elsewhere is refused by the older rule: install.sh writes $HOME's)."""
    user = _account(tmp_path, monkeypatch)
    sandbox, home = _sandbox_home(tmp_path, monkeypatch, roborun_home)
    _sandbox_install(sandbox)
    monkeypatch.delenv('MANIFEST_DREAMAPI', raising=False)
    assert robot.second_install()
    for command in ('up', 'setup'):
        robot.refuse_default_install(command, robot.home_paths())
    robot.refuse_default_install('down', robot.down_paths())
    robot.refuse_default_install('upgrade', robot.installer_paths() + robot.home_paths())


def test_uninstall_in_a_sandbox_whose_startup_file_is_the_account_s_changes_nothing(tmp_path, monkeypatch, systemd, capsys):
    """A sandbox with its own install, whose .bashrc is a symlink to the account's: removing the PATH line would edit the
    account's file. uninstall stops before stopping, editing or deleting anything."""
    user = _account(tmp_path, monkeypatch)
    sandbox, home = _sandbox_home(tmp_path, monkeypatch, 'unset')
    root = _sandbox_install(sandbox)
    (sandbox / '.bashrc').unlink()
    (sandbox / '.bashrc').symlink_to(user / '.bashrc')
    monkeypatch.setattr(sys, 'prefix', str(root / 'python'))
    with pytest.raises(SystemExit, match='roborun uninstall stopped.*Nothing was changed'):
        robot.cmd_uninstall(SimpleNamespace(yes=True, keep_data=False, purge=True, park=False, release=False))
    assert root.is_dir() and home.is_dir() and (sandbox / '.local' / 'bin' / 'roborun').is_symlink()
    _default_install_untouched(user)


def test_an_empty_roborun_home_is_the_default_home_not_the_current_folder(tmp_path):
    env = dict({k: v for k, v in os.environ.items() if k != 'ROBORUN_HOME'}, PYTHONPATH=str(SRC), ROBORUN_HOME='',
               HOME=str(tmp_path / 'h'))
    out = subprocess.run([sys.executable, '-c', 'from roborun_sdk import robot; print(robot.HOME)'], cwd=tmp_path,
                         env=env, capture_output=True, text=True, check=True).stdout.strip()
    assert out == str(tmp_path / 'h' / '.config' / 'roborun')


# ------------------------------------------------------------------------------------------------ down, a borrowed .config, relative homes
def test_down_of_a_second_install_whose_station_files_are_the_default_s_asks_nothing(tmp_path, monkeypatch, systemd):
    """ROBORUN_HOME=<second>, with <second>/robot a symlink to the default home's robot folder and this install's own
    station running: down would write {"do": "down"} into the default station's request file and signal its pid (the live
    station lets go of its arms). Refused before the station is asked anything."""
    user = _account(tmp_path, monkeypatch)
    live = user / '.config' / 'roborun' / 'robot'
    live.mkdir()
    (live / 'station-ready.json').write_text(json.dumps(dict(pid=os.getpid())))
    monkeypatch.setattr(robot.Path, 'home', classmethod(lambda cls: user))
    home = tmp_path / 'second'
    home.mkdir()
    (home / 'robot').symlink_to(live)
    monkeypatch.setattr(robot, 'HOME', home)
    monkeypatch.setattr(robot, 'CONFIG', home / 'robot.json')
    monkeypatch.setenv('ROBORUN_HOME', str(home))
    monkeypatch.setattr(robot, '_state', lambda name: 'active')            # this install's own station is running
    signalled = []
    monkeypatch.setattr(os, 'kill', lambda pid, sig: signalled.append((pid, sig)))
    assert robot.second_install()
    for flags in (dict(park=False, release=False), dict(park=True, release=False), dict(park=False, release=True)):
        with pytest.raises(SystemExit, match='roborun down stopped.*robot.*Nothing was changed'):
            robot.cmd_down(SimpleNamespace(**flags))
    assert not signalled and not systemd()
    assert sorted(p.name for p in live.iterdir()) == ['station-ready.json']        # no request was written


@pytest.mark.parametrize('linked', ['run', 'robot', 'logs', 'robot.json'])
def test_down_paths_cover_every_file_down_touches(tmp_path, monkeypatch, linked):
    user = _account(tmp_path, monkeypatch)
    monkeypatch.setattr(robot.Path, 'home', classmethod(lambda cls: user))
    target = user / '.config' / 'roborun' / linked
    if linked != 'robot.json':
        target.mkdir()
    home = tmp_path / 'second'
    home.mkdir()
    (home / linked).symlink_to(target)
    monkeypatch.setattr(robot, 'HOME', home)
    monkeypatch.setattr(robot, 'CONFIG', home / 'robot.json')
    monkeypatch.setenv('ROBORUN_HOME', str(home))
    with pytest.raises(SystemExit, match='roborun down stopped'):
        robot.refuse_default_install('down', robot.down_paths())


def _borrowed_config(tmp_path, monkeypatch, shape, roborun_home='unset'):
    """A sandbox with its own $HOME whose .config (or .config/roborun) is a symlink to the account's."""
    user = _account(tmp_path, monkeypatch)
    (user / '.config' / 'roborun' / 'robot').mkdir()
    sandbox = tmp_path / 'sandbox'
    sandbox.mkdir()
    if shape == '.config':
        (sandbox / '.config').symlink_to(user / '.config')
    else:
        (sandbox / '.config').mkdir()
        (sandbox / '.config' / 'roborun').symlink_to(user / '.config' / 'roborun')
    monkeypatch.setattr(robot.Path, 'home', classmethod(lambda cls: sandbox))
    home = sandbox / '.config' / 'roborun'
    monkeypatch.setattr(robot, 'HOME', home)
    monkeypatch.setattr(robot, 'CONFIG', home / 'robot.json')
    if roborun_home == 'unset':
        monkeypatch.delenv('ROBORUN_HOME', raising=False)
    else:
        monkeypatch.setenv('ROBORUN_HOME', str(home))
    monkeypatch.delenv('MANIFEST_DREAMAPI', raising=False)
    return user, sandbox


@pytest.mark.parametrize('roborun_home', ['unset', 'set'])
@pytest.mark.parametrize('shape', ['.config', '.config/roborun'])
def test_a_sandbox_borrowing_the_account_s_settings_is_a_second_install_and_is_refused(tmp_path, monkeypatch, systemd, capsys,
                                                                                       shape, roborun_home):
    """HOME=/tmp/sbx with /tmp/sbx/.config (or .config/roborun) a symlink to the account's: its home resolves to the default
    one, but it is not the default install (a sandbox that changes $HOME is a second install). Before, it got the plain unit
    names, so down stopped the live robot's units and uninstall --purge deleted the account's settings and key. Now it gets
    its own names, and up, down, upgrade and uninstall stop with nothing changed."""
    user, sandbox = _borrowed_config(tmp_path, monkeypatch, shape, roborun_home)
    assert robot._real(robot.HOME) == robot._real(robot.account_home())
    assert robot.second_install() and robot.install_suffix() != ''
    assert all(robot.unit_name(u) not in DEFAULT_UNITS for u in robot.UNITS)
    before = sorted(str(p) for p in user.rglob('*'))
    a = SimpleNamespace(manifest=None, sim=None, profile=None, rig=None, robot='r', key_file=None, server=None, detect=False)
    with pytest.raises(SystemExit, match='roborun up stopped.*Nothing was changed'):
        robot.cmd_up(a)
    with pytest.raises(SystemExit, match='roborun down stopped.*Nothing was changed'):
        robot.cmd_down(SimpleNamespace(park=False, release=False))
    with monkeypatch.context() as m:
        ran = _no_network(m)
        with pytest.raises(SystemExit, match='Nothing was changed'):
            robot.cmd_upgrade(SimpleNamespace(force=True))
        assert not ran
    monkeypatch.setattr(sys, 'prefix', str(user / '.roborun' / 'python'))     # the account's installed roborun
    with pytest.raises(SystemExit, match='roborun uninstall stopped.*Nothing was changed'):
        robot.cmd_uninstall(SimpleNamespace(yes=True, keep_data=False, purge=True, park=False, release=False))
    assert sorted(str(p) for p in user.rglob('*')) == before
    _default_install_untouched(user)
    assert not set(_named_units(systemd())) & DEFAULT_UNITS


def test_the_default_home_named_as_written_or_from_the_account_s_own_home_is_still_the_default_install(tmp_path, monkeypatch):
    """ROBORUN_HOME=/home/<you>/.config/roborun (that path itself, not a symlink to it) is the default install from any
    $HOME, as documented; so is the account's own $HOME whose .config is a symlink (dotfiles), and a ROBORUN_HOME that is a
    symlink to the default home when $HOME is the account's own folder."""
    user = _account(tmp_path, monkeypatch)
    sandbox = tmp_path / 'sandbox'
    sandbox.mkdir()
    monkeypatch.setattr(robot.Path, 'home', classmethod(lambda cls: sandbox))
    monkeypatch.setattr(robot, 'HOME', user / '.config' / 'roborun')
    monkeypatch.setenv('ROBORUN_HOME', str(user / '.config' / 'roborun'))
    assert robot.install_suffix() == ''
    link = tmp_path / 'link-to-default'
    link.symlink_to(user / '.config' / 'roborun')
    monkeypatch.setattr(robot, 'HOME', link)
    monkeypatch.setenv('ROBORUN_HOME', str(link))
    assert robot.install_suffix() != ''                     # from a sandbox $HOME: a borrowed home, a second install
    monkeypatch.setattr(robot.Path, 'home', classmethod(lambda cls: user))
    assert robot.install_suffix() == ''                     # from the account's own $HOME: the default install
    dotfiles = tmp_path / 'dotfiles'
    (dotfiles / 'roborun').mkdir(parents=True)
    acct = tmp_path / 'acct'
    acct.mkdir()
    (acct / '.config').symlink_to(dotfiles)
    monkeypatch.setattr(robot, 'account_home', lambda: acct / '.config' / 'roborun')
    monkeypatch.setattr(robot.Path, 'home', classmethod(lambda cls: acct))
    monkeypatch.setattr(robot, 'HOME', acct / '.config' / 'roborun')
    monkeypatch.delenv('ROBORUN_HOME', raising=False)
    assert robot.install_suffix() == '' and not robot.second_install()


@pytest.mark.parametrize('value', ['.config/roborun', 'second', '~/second'])
def test_a_relative_roborun_home_is_made_absolute_for_the_units(tmp_path, value):
    """HOME=/tmp/sbx, cwd /tmp/sbx, ROBORUN_HOME=.config/roborun (or a quoted ~): a systemd --user unit runs in the account's
    home folder, so a relative ROBORUN_HOME would name the default install's home there. Every module's home is absolute,
    and so is what the units get (--setenv=ROBORUN_HOME=..., MANIFEST_DREAMAPI)."""
    sbx = tmp_path / 'sbx'
    sbx.mkdir()
    stubs = tmp_path / 'stubs'
    stubs.mkdir()
    log = tmp_path / 'calls.log'
    log.write_text('')
    for name in ('systemctl', 'systemd-run'):
        (stubs / name).write_text(STUB.format(name=name, log=log))
        (stubs / name).chmod(0o755)
    env = dict({k: v for k, v in os.environ.items() if k not in ('ROBORUN_HOME', 'MANIFEST_DREAMAPI', 'DBUS_SESSION_BUS_ADDRESS')},
               PYTHONPATH=str(SRC), HOME=str(sbx), ROBORUN_HOME=value, PATH=f"{stubs}{os.pathsep}{os.environ.get('PATH', '')}",
               XDG_RUNTIME_DIR=str(tmp_path))
    code = '''
import json
from roborun_sdk import robot, net, dataset, devkit
from roborun_sdk.arms import yam, safety, direction_check
robot._unit('roborun-robot', ['true'], dict(ROBORUN_HOME=str(robot.HOME), MANIFEST_DREAMAPI=robot._manifest_root()))
print(json.dumps(dict(home=str(robot.HOME), config=str(robot.CONFIG), net=str(net.roborun_home()), data=str(robot.data_top()),
                      runtime=robot._manifest_root(), dataset=str(dataset._state_file()), devkit=str(devkit._config_home()),
                      yam=str(yam._state_dir()), safety=str(safety._home()), checks=str(direction_check._result_file('a', 'b')))))
'''
    out = json.loads(subprocess.run([sys.executable, '-c', code], cwd=sbx, env=env, capture_output=True, text=True,
                                    check=True).stdout)
    want = (sbx / value[2:]) if value.startswith('~/') else (sbx / value)
    assert out['home'] == out['net'] == out['safety'] == out['devkit'] == str(want)
    assert all(os.path.isabs(v) and '~' not in v for v in out.values())
    assert out['config'] == str(want / 'robot.json')
    calls = [line.split() for line in log.read_text().splitlines() if line.startswith('systemd-run')]
    assert len(calls) == 1
    setenv = dict(a[len('--setenv='):].split('=', 1) for a in calls[0] if a.startswith('--setenv='))
    assert setenv['ROBORUN_HOME'] == str(want) and os.path.isabs(setenv['MANIFEST_DREAMAPI'])
    assert re.fullmatch(r'--unit=roborun-robot-[0-9a-f]{8}', [a for a in calls[0] if a.startswith('--unit=')][0])


@pytest.mark.parametrize('how', ['token-link', 'key-file'])
def test_login_of_a_second_install_never_writes_the_default_install_s_key(tmp_path, monkeypatch, how):
    """A second home whose token is a symlink to the account's (or --key-file naming the account's key): the approved
    sign-in would replace the running robot's key. Refused before the API is asked for a code."""
    from roborun_sdk import login
    user = _account(tmp_path, monkeypatch)
    (user / '.config' / 'roborun' / 'token').write_text('LIVEKEY')
    monkeypatch.setattr(robot.Path, 'home', classmethod(lambda cls: user))
    home = tmp_path / 'second'
    home.mkdir()
    monkeypatch.setattr(robot, 'HOME', home)
    monkeypatch.setattr(robot, 'CONFIG', home / 'robot.json')
    monkeypatch.setenv('ROBORUN_HOME', str(home))
    key_file = None
    if how == 'token-link':
        (home / 'token').symlink_to(user / '.config' / 'roborun' / 'token')
    else:
        key_file = str(user / '.config' / 'roborun' / 'token')
    asked = []
    monkeypatch.setattr(login, '_post', lambda *a, **k: asked.append(a))
    with pytest.raises(SystemExit, match='roborun login stopped.*Nothing was changed'):
        login.cmd_login(SimpleNamespace(key_file=key_file, wait=False, server='https://api.example', name=None))
    assert not asked and (user / '.config' / 'roborun' / 'token').read_text() == 'LIVEKEY'
    assert not (user / '.config' / 'roborun' / 'login-pending.json').exists()
