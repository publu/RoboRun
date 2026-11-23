"""What a new user meets around the robot itself: the last lines of `roborun up`, keeping it running after logout, a missing
OpenGL library said before any setup, `roborun status` naming the next step, `roborun uninstall`, the key prompt that offers the
browser sign-in, the help that lists the user's commands, doctor's one-line summary, the client's `body_run`."""
import json
import re
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from roborun_sdk import cli, robot


@pytest.fixture()
def home(tmp_path, monkeypatch):
    cfg = tmp_path / 'config'
    cfg.mkdir()
    monkeypatch.setattr(robot, 'HOME', cfg)
    monkeypatch.setattr(robot, 'CONFIG', cfg / 'robot.json')
    monkeypatch.setattr(robot.net, 'update_notice', lambda *a, **k: None)
    monkeypatch.delenv('ROBORUN_NO_LINGER', raising=False)
    return tmp_path


# ------------------------------------------------------------------------------------------------ staying up after logout
def _systemd_with_linger(monkeypatch, linger_dir, allowed=True):
    calls = []
    monkeypatch.setattr(robot, '_systemd', lambda: True)
    monkeypatch.setattr(robot.shutil, 'which', lambda name: '/usr/bin/' + name)
    monkeypatch.setattr(robot, 'LINGER_DIR', linger_dir)
    monkeypatch.setattr(robot, '_user', lambda: 'alice')

    def run(argv, **k):
        calls.append(argv)
        if argv[:2] == ['loginctl', 'enable-linger'] and allowed:
            (linger_dir / argv[2]).write_text('')
        if argv[:2] == ['loginctl', 'disable-linger']:
            (linger_dir / argv[2]).unlink(missing_ok=True)
        return subprocess.CompletedProcess(argv, 0 if allowed else 1, '', '')
    monkeypatch.setattr(subprocess, 'run', run)
    return calls


def test_up_keeps_the_robot_running_after_logout_and_says_so(home, monkeypatch):
    linger = home / 'linger'; linger.mkdir()
    calls = _systemd_with_linger(monkeypatch, linger)
    said = []
    robot.ready(dict(backend='http://127.0.0.1:8120', live_port=8135), say=said.append)
    assert ['loginctl', 'enable-linger', 'alice'] in calls and (linger / 'alice').exists()
    assert any('kept running after you log out' in s for s in said) and (robot.HOME / robot.LINGER_MARK).is_file()
    assert said[-2] == "ready: open https://app.roborun.app on this computer, or this robot's own viewer: http://127.0.0.1:8135/"
    assert said[-1].startswith('local API: http://127.0.0.1:8120/v1/docs') and 'roborun docs agents' in said[-1]
    calls.clear(); said.clear()
    robot.ready(dict(backend='http://127.0.0.1:8120'), say=said.append)        # already on: asks nothing, says nothing about it
    assert calls == [] and not any('log out' in s for s in said)


def test_when_linger_cannot_be_turned_on_up_says_how(home, monkeypatch):
    linger = home / 'linger'; linger.mkdir()
    _systemd_with_linger(monkeypatch, linger, allowed=False)
    said = []
    robot.keep_running_after_logout(said.append)
    assert said == ['note: Roborun stops when you log out of this computer. To keep it running (an administrator command): '
                    'sudo loginctl enable-linger alice']
    monkeypatch.setenv('ROBORUN_NO_LINGER', '1')
    calls = _systemd_with_linger(monkeypatch, linger)
    said.clear()
    robot.keep_running_after_logout(said.append)
    assert calls == [] and 'sudo loginctl enable-linger alice' in said[0]


def test_without_systemd_there_is_nothing_to_keep(home, monkeypatch):
    monkeypatch.setattr(robot, '_systemd', lambda: False)
    said = []
    robot.keep_running_after_logout(said.append)
    assert said == [] and robot.linger_state() is None


def test_every_profile_ends_up_with_the_same_ready_lines():
    arm = (Path(robot.__file__).read_text())
    body = (Path(robot.__file__).parent / 'body' / 'up.py').read_text()
    assert 'ready(c)' in arm and 'robot.ready(c)' in body
    assert 'start this profile\'s station the way it needs' not in arm and 'docs/BUILDING.md)' not in body


# ------------------------------------------------------------------------------------------------ a missing OpenGL library
def test_a_missing_opengl_library_is_said_before_any_setup(home, monkeypatch):
    import ctypes.util
    monkeypatch.delenv('MUJOCO_GL', raising=False)
    monkeypatch.setattr(ctypes.util, 'find_library', lambda name: None)
    monkeypatch.setattr(sys, 'platform', 'linux')
    from roborun_sdk.arms import settings

    def no_setup(a):
        raise AssertionError('setup ran before the OpenGL check')
    monkeypatch.setattr(settings, 'first_run', no_setup)
    a = SimpleNamespace(sim='cup', key_file=None)
    with pytest.raises(SystemExit) as e:
        robot.cmd_up(a)
    text = str(e.value)
    assert 'sudo apt install libosmesa6' in text and 'mesa-libOSMesa' in text and 'administrator' in text
    assert not robot.CONFIG.exists()


# ------------------------------------------------------------------------------------------------ status says what next
def test_status_of_a_stopped_robot_names_the_next_step(home, monkeypatch, capsys):
    (home / 'token').write_text('rr_test_x')
    robot.CONFIG.write_text(json.dumps(dict(server='http://127.0.0.1:9', robot='bench', key_file=str(home / 'token'),
                                            backend='http://127.0.0.1:9', live_port=0, data=dict(enabled=False))))
    monkeypatch.setattr(robot, '_state', lambda u: 'inactive')
    monkeypatch.setattr(robot, 'linger_state', lambda: False)
    monkeypatch.setattr(robot, '_user', lambda: 'alice')
    monkeypatch.setattr(robot, 'token_state', lambda server, key: (None, 'not reachable'))
    assert robot.cmd_status(None) == 0
    out = capsys.readouterr().out
    assert 'next: roborun up' in out and 'after logout      stops (to keep it running: sudo loginctl enable-linger alice)' in out


# ------------------------------------------------------------------------------------------------ uninstall
def _installed(home, monkeypatch):
    user = home / 'user'
    root = user / '.roborun'
    (root / 'python' / 'bin').mkdir(parents=True)
    (root / 'tools').mkdir()
    (root / 'python' / 'bin' / 'roborun').write_text('#!/bin/sh\n')
    (user / '.local' / 'bin').mkdir(parents=True)
    (user / '.local' / 'bin' / 'roborun').symlink_to(root / 'python' / 'bin' / 'roborun')
    (user / '.bashrc').write_text('alias ll="ls -l"\n' + robot.RC_BLOCK + 'export EDITOR=vi\n')
    runtime = user / 'roborun' / 'robot' / 'runtime'
    (runtime / 'log').mkdir(parents=True)
    (runtime / 'RUNTIME.json').write_text('{"version": "x"}')
    (runtime / 'log' / 'recording.jpg').write_bytes(b'jpg')
    (robot.HOME / 'token').write_text('rr_test_x')
    robot.CONFIG.write_text(json.dumps(dict(manifest=str(runtime))))
    monkeypatch.setattr(robot, 'install_root', lambda: root)
    monkeypatch.setattr(robot.Path, 'home', classmethod(lambda cls: user))
    monkeypatch.setattr(robot, '_state', lambda u: 'inactive')
    monkeypatch.setattr(sys.stdin, 'isatty', lambda: False)
    return user, root, runtime


def _uninstall(**kw):
    return robot.cmd_uninstall(SimpleNamespace(**dict(dict(yes=True, keep_data=False, purge=False, park=False, release=False), **kw)))


def test_uninstall_removes_roborun_and_keeps_the_person_s_data(home, monkeypatch, capsys):
    user, root, runtime = _installed(home, monkeypatch)
    assert _uninstall(keep_data=True) == 0
    out = capsys.readouterr().out
    assert not root.exists() and not (user / '.local' / 'bin' / 'roborun').exists()
    assert (user / '.bashrc').read_text() == 'alias ll="ls -l"\n\nexport EDITOR=vi\n'
    assert (robot.HOME / 'token').is_file() and (runtime / 'log' / 'recording.jpg').is_file()
    assert 'Roborun is removed from this computer.' in out and 'Kept (your settings, key and recordings)' in out


def test_uninstall_purge_deletes_settings_and_recordings_too(home, monkeypatch, capsys):
    user, root, runtime = _installed(home, monkeypatch)
    assert _uninstall(purge=True) == 0
    assert not root.exists() and not robot.HOME.exists() and not (user / 'roborun').exists()


def test_uninstall_asks_first_and_never_drops_a_held_arm(home, monkeypatch):
    user, root, runtime = _installed(home, monkeypatch)
    with pytest.raises(SystemExit, match='roborun uninstall --yes'):
        _uninstall(yes=False)                                     # no terminal to ask in, no --yes: nothing removed
    assert root.exists()
    monkeypatch.setattr(robot, '_state', lambda u: 'active' if u == 'roborun-station' else 'inactive')
    monkeypatch.setattr(robot, 'cmd_down', lambda a: 0)           # the station keeps a held arm powered
    with pytest.raises(SystemExit, match='an arm is still powered and held'):
        _uninstall()
    assert root.exists() and (user / '.local' / 'bin' / 'roborun').exists()


def test_uninstall_turns_off_only_the_linger_roborun_turned_on(home, monkeypatch, capsys):
    user, root, runtime = _installed(home, monkeypatch)
    linger = home / 'linger'; linger.mkdir(); (linger / 'alice').write_text('')
    calls = _systemd_with_linger(monkeypatch, linger)
    monkeypatch.setattr(robot, '_state', lambda u: 'inactive')
    (robot.HOME / robot.LINGER_MARK).write_text('alice\n')
    assert _uninstall(keep_data=True) == 0
    assert ['loginctl', 'disable-linger', 'alice'] in calls


def test_uninstall_and_view_are_commands(capsys):
    with pytest.raises(SystemExit):
        cli.main(['uninstall', '--help'])
    text = capsys.readouterr().out
    assert '--keep-data' in text and '--purge' in text and '--yes' in text


# ------------------------------------------------------------------------------------------------ the key prompt
def test_the_key_prompt_offers_the_browser_sign_in(tmp_path, monkeypatch):
    from roborun_sdk import login
    from roborun_sdk.arms import settings
    signed = []
    monkeypatch.delenv('ROBORUN_API_KEY', raising=False); monkeypatch.delenv('ROBORUN_TOKEN', raising=False)
    monkeypatch.setattr(sys.stdin, 'isatty', lambda: True)
    asked = []
    monkeypatch.setattr(settings.getpass, 'getpass', lambda prompt: (asked.append(prompt), '')[1])
    monkeypatch.setattr(login, 'login', lambda server, file: signed.append((server, file)))
    settings._ask_token(tmp_path / 'token', 'https://api.example')
    assert 'press Enter to sign in with your browser' in asked[0] and signed == [('https://api.example', tmp_path / 'token')]
    monkeypatch.setattr(sys.stdin, 'isatty', lambda: False)
    with pytest.raises(SystemExit) as e:
        settings._ask_token(tmp_path / 'token', 'https://api.example')
    assert 'roborun login' in str(e.value) and '--key-file' in str(e.value) and 'ROBORUN_API_KEY' in str(e.value)


# ------------------------------------------------------------------------------------------------ help and doctor
def test_the_help_lists_the_user_s_commands_and_hides_the_developer_ones(capsys):
    with pytest.raises(SystemExit):
        cli.main(['--help'])
    short = capsys.readouterr().out
    with pytest.raises(SystemExit):
        cli.main(['--help-all'])
    full = capsys.readouterr().out

    def listed(name, text):     # a command row; a long name has its help on the next line (Python 3.10 and 3.12 wrap differently)
        return re.search(rf'^ {{4}}{re.escape(name)}(?: |$)', text, re.M) is not None

    for name in cli.DEVELOPER_COMMANDS:
        assert not listed(name, short) and listed(name, full), name
    for name in ('up', 'down', 'status', 'stop', 'login', 'uninstall', 'view', 'docs'):
        assert listed(name, short), name
    assert 'roborun docs agents' in short and 'galaxea_r1 --sim room' in short and '--help-all' in short


def test_doctor_starts_with_a_plain_summary(capsys):
    assert cli.main(['doctor']) == 0
    out = capsys.readouterr().out
    assert out.startswith('OK: ') and 'import_guard' not in out and 'roborun doctor --verbose' in out


# ------------------------------------------------------------------------------------------------ the client
def test_body_run_waits_for_the_move_and_stops_it_when_interrupted(monkeypatch):
    from roborun_sdk.client import Robot
    r = Robot('http://127.0.0.1:9')
    views = iter([dict(id='mv-1', status='running', stage='planning'), dict(id='mv-1', status='running', stage='moving'),
                  dict(id='mv-1', status='done', stage='done')])
    calls = []
    monkeypatch.setattr(r, 'body_move', lambda prompt, duration_s=4: calls.append(('move', prompt, duration_s)) or dict(id='mv-1'))
    monkeypatch.setattr(r, 'body_move_status', lambda i: next(views))
    monkeypatch.setattr(r, 'stop', lambda: calls.append(('stop',)))
    seen = []
    out = r.body_run('walk', duration_s=2, poll_s=0, on_update=seen.append)
    assert out['status'] == 'done' and [v['stage'] for v in seen] == ['planning', 'moving', 'done'] and ('stop',) not in calls
    monkeypatch.setattr(r, 'body_move_status', lambda i: dict(id='mv-1', status='running', stage='moving'))
    with pytest.raises(TimeoutError):
        r.body_run('walk', timeout=0, poll_s=0)
    assert calls[-1] == ('stop',)


def test_profiles_have_readable_labels_and_say_how_to_simulate(home, capsys):
    assert robot.cmd_profiles(SimpleNamespace(manifest=str(home / 'no-runtime'))) == 0
    out = capsys.readouterr().out
    assert '  yam          one YAM arm' in out and '  yam_dual     two YAM arms' in out and 'pre-made' not in out
    assert 'yam_metal    yam_metal' not in out and 'yam_single' not in out and 'makerarm' not in out
    assert '--sim cup, pickup-bench, mixed-objects, cluttered-bench, low-light' in out and '--sim room or empty' in out and 'galaxea_r1' in out


def test_the_simulator_keeps_its_shader_cache_inside_roborun(tmp_path, monkeypatch):
    import ctypes.util
    monkeypatch.delenv('MUJOCO_GL', raising=False); monkeypatch.delenv('MESA_SHADER_CACHE_DIR', raising=False)
    monkeypatch.setattr(ctypes.util, 'find_library', lambda name: 'libOSMesa.so.8' if name == 'OSMesa' else None)
    monkeypatch.setattr(robot, 'install_root', lambda: tmp_path / '.roborun')
    env = robot.gl_env()
    assert env['MUJOCO_GL'] == 'osmesa' and env['MESA_SHADER_CACHE_DIR'] == str(tmp_path / '.roborun' / 'cache' / 'mesa')
