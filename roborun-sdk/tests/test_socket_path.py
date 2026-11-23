"""The camera sockets fit in a Unix socket path: one rule (robot.socket_path) for the camera service, the simulation, roborun up
and every reader, which falls back to a short folder of this user's in /tmp when the usual folder (a deep $XDG_RUNTIME_DIR, a
second install's folder on top) has no room for its sockets within what AF_UNIX allows. Nothing here starts a service."""
import os
import shutil
import socket
import stat
import subprocess
import sys
import tempfile
import threading
from pathlib import Path

import pytest

from roborun_sdk import cameras, robot, sim_home

SRC = Path(__file__).resolve().parents[1] / 'src'
LONG_TAIL = len('/manifest/roborun-12345678/camera.sock')


@pytest.fixture
def install(tmp_path, monkeypatch):
    """A second install: its own $HOME and Roborun home, never the account's. Returns a function that sets the runtime dir."""
    user = tmp_path / 'user'; user.mkdir()
    monkeypatch.setenv('HOME', str(user))
    for k in ('ROBORUN_HOME', 'MANIFEST_CAMERA_SOCKET'):
        monkeypatch.delenv(k, raising=False)
    monkeypatch.setattr(robot, 'HOME', user / '.config' / 'roborun')
    monkeypatch.setattr(robot, 'CONFIG', user / '.config' / 'roborun' / 'robot.json')
    monkeypatch.setattr(robot, 'account_home', lambda: tmp_path / 'account' / '.config' / 'roborun')
    named = set()
    short = robot.short_socket_folder
    monkeypatch.setattr(robot, 'short_socket_folder', lambda p: named.add(short(p)) or short(p))

    def runtime_dir(path):
        Path(path).mkdir(parents=True, exist_ok=True); os.chmod(path, 0o700)
        monkeypatch.setenv('XDG_RUNTIME_DIR', str(path))
        return Path(path)
    yield runtime_dir
    for folder in map(Path, named):                                     # the short folders this test made in /tmp
        if folder.is_dir() and not folder.is_symlink():
            shutil.rmtree(folder, ignore_errors=True)


@pytest.fixture
def short_dir():
    d = tempfile.mkdtemp(prefix='x', dir='/tmp')
    yield d
    shutil.rmtree(d, ignore_errors=True)


def deep(tmp_path):
    """A runtime dir like a sandbox's: <sandboxes>/<name>/run/xdg, deep enough that a second install's socket path is too long."""
    d = tmp_path / 'sandboxes' / ('a-sandbox-named-for-what-it-tests' + 'x' * 40) / 'run' / 'xdg'
    assert len(os.fsencode(str(d))) + LONG_TAIL > robot.SOCKET_MAX
    return d


def test_a_short_runtime_dir_keeps_the_usual_paths(install, short_dir, monkeypatch):
    run = install(short_dir)
    suffix = robot.install_suffix()
    assert suffix                                                       # a second install: its own folder
    assert robot.camera_socket() == str(run / 'manifest' / ('roborun' + suffix) / 'camera.sock') == cameras._socket_path()
    assert sim_home.camera_socket() == str(run / 'manifest' / ('roborun' + suffix) / 'sim-camera.sock')
    monkeypatch.setattr(robot, 'account_home', lambda: robot.HOME)     # the account's default install: as always
    assert robot.install_suffix() == ''
    assert robot.camera_socket() == str(run / 'manifest' / 'camera.sock') == cameras._socket_path()
    assert sim_home.camera_socket() == str(run / 'manifest' / 'sim-camera.sock')


def test_a_deep_runtime_dir_and_a_second_install_get_a_short_socket_everyone_finds(install, tmp_path):
    run = install(deep(tmp_path))
    usual = run / 'manifest' / ('roborun' + robot.install_suffix()) / 'camera.sock'
    assert len(os.fsencode(str(usual))) > robot.SOCKET_MAX               # the old path: bind() fails with "AF_UNIX path too long"
    cam = robot.camera_socket()
    folder = Path(cam).parent
    assert len(os.fsencode(cam)) <= robot.SOCKET_MAX and cam.endswith('/camera.sock')
    assert folder.parent == Path('/tmp') and folder.name.startswith(f'rr-{os.getuid()}-')
    st = os.lstat(folder)
    assert stat.S_ISDIR(st.st_mode) and st.st_uid == os.getuid() and stat.S_IMODE(st.st_mode) == 0o700
    assert cameras._socket_path() == cam and robot.camera_socket() == cam      # the reader's, without making anything: the same
    assert sim_home.robot_socket(cam)                                   # still the robot's socket: bare `roborun sim` refuses it
    sim = sim_home.camera_socket()
    assert Path(sim).parent == folder and sim != cam and not sim_home.robot_socket(sim)     # side by side, as usual
    helper = cameras.helper_socket(cam)                                 # the RealSense capture helper's, beside camera.sock
    assert Path(helper).parent == folder and len(os.fsencode(helper)) <= robot.SOCKET_MAX

    srv = socket.socket(socket.AF_UNIX); srv.bind(cam); srv.listen(1)       # the camera service, serving there
    threading.Thread(target=lambda: srv.accept()[0].sendall(b'frame'), daemon=True).start()
    try:
        reader = ('import socket; from roborun_sdk import robot; s = socket.socket(socket.AF_UNIX); '
                  's.connect(robot.camera_socket()); print(robot.camera_socket(), s.recv(5).decode())')
        out = subprocess.run([sys.executable, '-c', reader], capture_output=True, text=True, timeout=60,
                             env=dict(os.environ, PYTHONPATH=str(SRC)))     # another process works it out on its own
        assert out.returncode == 0, out.stderr
        assert out.stdout.split() == [cam, 'frame']
    finally:
        srv.close()


def test_a_folder_with_room_for_camera_sock_but_not_its_helper_s_socket_moves_them_all(install, short_dir):
    folder_len = robot.SOCKET_MAX - len('/camera-capture.sock') + 1      # one byte too deep for camera-capture.sock
    pad = folder_len - len(os.fsencode(short_dir)) - len('/manifest/roborun-12345678') - 1
    run = install(Path(short_dir) / ('p' * pad))
    usual = run / 'manifest' / ('roborun' + robot.install_suffix())
    assert len(os.fsencode(str(usual))) == folder_len and len(os.fsencode(str(usual / 'camera.sock'))) <= robot.SOCKET_MAX
    cam = robot.camera_socket()
    assert Path(cam).parent.parent == Path('/tmp') and Path(sim_home.camera_socket()).parent == Path(cam).parent
    assert len(os.fsencode(cameras.helper_socket(cam))) <= robot.SOCKET_MAX


def test_each_install_and_each_runtime_dir_has_its_own_short_folder(install, tmp_path, monkeypatch):
    run = install(deep(tmp_path))
    first = robot.camera_socket()
    other = tmp_path / 'other'; other.mkdir()
    monkeypatch.setenv('HOME', str(other)); monkeypatch.setattr(robot, 'HOME', other / '.config' / 'roborun')
    second = robot.camera_socket()
    install(run.parent / 'xdg2')
    third = robot.camera_socket()
    assert len({first, second, third}) == 3 and len({Path(p).parent for p in (first, second, third)}) == 3


def test_manifest_camera_socket_still_names_the_socket(install, tmp_path, monkeypatch):
    install(deep(tmp_path))
    monkeypatch.setenv('MANIFEST_CAMERA_SOCKET', '/run/cams/robot.sock')
    assert robot.camera_socket() == cameras._socket_path() == '/run/cams/robot.sock'


def test_a_short_folder_this_user_does_not_own_is_refused(install, tmp_path):
    install(deep(tmp_path))
    folder = Path(robot.socket_path('camera.sock')).parent
    elsewhere = tmp_path / 'someone-else'; elsewhere.mkdir()
    folder.symlink_to(elsewhere)                                         # made first by someone else, leading elsewhere
    try:
        with pytest.raises(SystemExit, match='not a folder of this user: it is not used'):
            robot.camera_socket()
        assert not list(elsewhere.iterdir())
    finally:
        folder.unlink()
    folder.mkdir(mode=0o700); os.chmod(folder, 0o777)                    # this user's, but open to others: closed again
    robot.camera_socket()
    assert stat.S_IMODE(os.lstat(folder).st_mode) == 0o700
