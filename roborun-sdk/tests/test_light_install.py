"""The light install: `pip install roborun-sdk` is the client, your own drivers and the `roborun` command; the heavy and
hardware packages are extras, [robot] is everything (what install.sh installs), importing never pulls numpy, MuJoCo or OpenCV
until something needs them, a missing one names its extra, and importing works where fcntl and grp do not exist (Windows)."""
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

try:
    import tomllib                                                          # Python 3.11+
except ModuleNotFoundError:
    tomllib = pytest.importorskip('tomli')                                  # Python 3.10: the test extra installs it

PKG = Path(__file__).resolve().parents[1]
SRC = PKG / 'src'
PYPROJECT = tomllib.loads((PKG / 'pyproject.toml').read_text())
# not imported by the light modules (cryptography is: rt.signing picks its Ed25519 backend at import, and falls back without it)
HEAVY = ('numpy', 'mujoco', 'cv2', 'pyrealsense2', 'msgpack', 'PIL', 'can', 'serial', 'pyarrow', 'av')
MISSING = HEAVY + ('cryptography', 'websockets', 'zstandard', 'pydantic')     # what a light install does not have
LIGHT_MODULES = ('roborun_sdk', 'roborun_sdk.client', 'roborun_sdk.errors', 'roborun_sdk.embodiment', 'roborun_sdk.arms',
                 'roborun_sdk.arms.base', 'roborun_sdk.arms.safety', 'roborun_sdk.body', 'roborun_sdk.cameras', 'roborun_sdk.rt',
                 'roborun_sdk.rt.signing', 'roborun_sdk.link', 'roborun_sdk.devkit', 'roborun_sdk.robot', 'roborun_sdk.dataset',
                 'roborun_sdk.mujoco_sim', 'roborun_sdk.profile_create', 'roborun_sdk.cli')
# what `pip install roborun-sdk` installed before the extras (pyproject.toml at e32e29a): [robot] must stay exactly this
OLD_BASE = ['numpy==2.2.6', 'msgpack>=1.0', 'websockets>=12', 'zstandard>=0.22', 'pillow>=10', 'cryptography>=42',
            'packaging>=23', 'pyyaml>=6', 'pydantic>=2', 'mujoco==3.17.0', 'opencv-python-headless>=4.8',
            'python-can>=4.4', 'pyserial>=3.5', 'pyusb>=1.2.1', 'libusb-package>=1.0.26',
            'tyro>=0.8', 'crcmod-plus>=2', 'dm-env>=1.6', 'mcap-ros2-support>=0.5']

# run Python with some packages made unimportable (as on a light install), optionally with the POSIX-only modules gone too
BLOCKER = r'''
import importlib.abc, os, sys
BLOCK = set(sys.argv[1].split(',')) - {''}
class Block(importlib.abc.MetaPathFinder):
    def find_spec(self, name, path=None, target=None):
        if name.split('.')[0] in BLOCK:
            raise ModuleNotFoundError(f"No module named '{name}'", name=name)
for m in list(sys.modules):
    if m.split('.')[0] in BLOCK: del sys.modules[m]
sys.meta_path.insert(0, Block())
if sys.argv[2] == 'windows':
    for a in ('getuid', 'geteuid', 'getgid', 'getgroups', 'getpgid', 'killpg', 'setsid'):
        if hasattr(os, a): delattr(os, a)
sys.argv = sys.argv[2:]
'''


def _python(code, block=(), windows=False, cwd=None):
    script = BLOCKER + code
    env = dict(os.environ, PYTHONPATH=str(SRC), PYTHONDONTWRITEBYTECODE='1')
    return subprocess.run([sys.executable, '-c', script, ','.join(block), 'windows' if windows else 'posix'],
                          capture_output=True, text=True, env=env, cwd=cwd or PKG, timeout=120)


def _names(reqs):
    from packaging.requirements import Requirement
    return {Requirement(r).name for r in reqs}


# ------------------------------------------------------------------------------------------------- packaging
def test_base_install_is_light_and_robot_extra_is_the_old_full_set():
    base = _names(PYPROJECT['project']['dependencies'])
    assert base == {'packaging', 'pyyaml', 'roborun-station'}
    extras = PYPROJECT['project']['optional-dependencies']
    assert extras['robot'] == OLD_BASE                          # install.sh installs the same packages and pins as before


def test_each_extra_brings_what_its_modules_import():
    extras = {k: _names(v) for k, v in PYPROJECT['project']['optional-dependencies'].items()}
    assert {'msgpack', 'numpy', 'websockets', 'zstandard', 'pillow'} <= extras['link']
    assert {'mujoco', 'numpy'} <= extras['sim'] and {'mujoco', 'numpy'} <= extras['urdf']
    assert {'numpy', 'opencv-python-headless', 'pyrealsense2'} <= extras['cameras']
    assert {'numpy', 'pyarrow', 'av'} <= extras['dataset']
    assert {'numpy', 'mujoco', 'python-can'} <= extras['yam']
    robot = extras['robot']
    for name in ('link', 'crypto', 'sim', 'urdf', 'yam', 'damiao', 'robstride', 'feetech', 'dynamixel'):
        assert extras[name] <= robot, name                       # [robot] has everything the arm and robot extras have


def test_the_missing_extra_hints_name_real_extras():
    from roborun_sdk import _optional
    extras = set(PYPROJECT['project']['optional-dependencies'])
    assert set(_optional.EXTRA_FOR.values()) <= extras
    assert _optional.install_hint('cv2') == "cv2 is not installed: pip install 'roborun-sdk[cameras]', or 'roborun-sdk[robot]' for everything a robot runs"
    assert _optional.install_hint('numpy') == "numpy is not installed: pip install 'roborun-sdk[robot]'"


# ---------------------------------------------------------------------------------------------- install.sh
def _fake_install(tmp_path, env_extra=None):
    """Run install.sh with a stand-in uv (records its arguments) and curl (serves a manifest and two wheels): no network."""
    home = tmp_path / 'home'
    tools = home / '.roborun' / 'tools'
    tools.mkdir(parents=True)
    log = tmp_path / 'uv.log'
    (tools / 'uv').write_text(f'''#!/bin/sh
printf '%s\\n' "$*" >> {log}
if [ "$1" = venv ]; then for a; do v="$a"; done; mkdir -p "$v/bin"; ln -sf {sys.executable} "$v/bin/python"
  printf '#!/bin/sh\\nexit 0\\n' > "$v/bin/roborun"; chmod +x "$v/bin/roborun"; fi
exit 0
''')
    (tools / 'uv').chmod(0o755)
    wheels = {'sdk': 'roborun_sdk-0.1.20990101000000-py3-none-any.whl', 'station': 'roborun_station-0.1.20990101000000-py3-none-any.whl'}
    blob = b'not really a wheel'
    manifest = dict(build='20990101-000000-aaaaaaaaaa', wheels={k: dict(file=v, sha256=hashlib.sha256(blob).hexdigest())
                                                                for k, v in wheels.items()})
    (tmp_path / 'manifest.json').write_text(json.dumps(manifest))
    (tmp_path / 'blob').write_bytes(blob)
    bin_ = tmp_path / 'bin'
    bin_.mkdir()
    (bin_ / 'curl').write_text(f'''#!/bin/sh
for a; do out="$a"; url="$prev"; prev="$a"; done
case "$*" in *-o*) ;; *) exit 0 ;; esac
case "$out" in *manifest.json) cp {tmp_path}/manifest.json "$out" ;; *) cp {tmp_path}/blob "$out" ;; esac
''')
    (bin_ / 'curl').chmod(0o755)
    env = dict(PATH=f'{bin_}:/usr/bin:/bin', HOME=str(home), SHELL='/bin/sh', ROBORUN_NO_MODIFY_PATH='1', **(env_extra or {}))
    out = subprocess.run(['sh', str(PKG / 'install.sh')], env=env, capture_output=True, text=True, stdin=subprocess.DEVNULL,
                         timeout=60, start_new_session=True)
    assert out.returncode == 0, out.stderr + out.stdout
    return [line for line in log.read_text().splitlines() if line.startswith('pip install')]


def test_install_sh_installs_the_robot_extra_from_the_release_wheels(tmp_path):
    (pip,) = _fake_install(tmp_path)
    args = pip.split()
    sdk = [a for a in args if 'roborun_sdk-' in a]
    station = [a for a in args if 'roborun_station-' in a]
    assert len(sdk) == 1 and sdk[0].endswith('roborun_sdk-0.1.20990101000000-py3-none-any.whl[robot]')
    assert len(station) == 1 and station[0].endswith('.whl')           # the station has no extras


def test_install_sh_installs_the_robot_extra_from_a_wheel_folder(tmp_path):
    (pip,) = _fake_install(tmp_path, {'ROBORUN_WHEELS': str(tmp_path)})
    assert pip.split()[-1] == 'roborun-sdk[robot]' and f'--find-links {tmp_path}' in pip


# ------------------------------------------------------------------------------------------------ imports
def test_importing_the_sdk_pulls_no_heavy_package():
    code = ('import importlib, sys\n'
            f'for m in {LIGHT_MODULES!r}: importlib.import_module(m)\n'
            f'print(sorted(h for h in {HEAVY!r} if h in sys.modules))')
    out = _python(code)
    assert out.returncode == 0, out.stderr
    assert out.stdout.strip() == '[]'


def test_a_light_install_imports_and_runs_roborun_help():
    code = ('import importlib, sys\n'
            f'for m in {LIGHT_MODULES!r}: importlib.import_module(m)\n'
            'from roborun_sdk import cli\n'
            'for argv in (["--help"], ["--help-all"], ["--version"]):\n'
            '    try: cli.main(argv)\n'
            '    except SystemExit as e: assert not e.code, (argv, e.code)\n')
    out = _python(code, block=MISSING)
    assert out.returncode == 0, out.stderr
    assert 'roborun login' in out.stdout


def test_a_missing_package_names_its_extra(tmp_path):
    code = ('from roborun_sdk import profile_create, cameras\n'
            'from roborun_sdk._optional import MissingExtra\n'
            'try: profile_create.convert("x.urdf", ".", "x", [], [], "tcp")\n'
            'except MissingExtra as e: print("URDF", e)\n'
            'try: cameras.np.zeros(1)\n'
            'except ImportError as e: print("CAM", e)\n'
            'try: import roborun_sdk.link.codec\n'
            'except ImportError as e: print("LINK", e)\n'
            'from roborun_sdk import cli\n'
            'print("RC", cli.main(["run", "seg.json"]))\n')
    out = _python(code, block=MISSING, cwd=tmp_path)
    assert out.returncode == 0, out.stderr
    assert "URDF mujoco is not installed: pip install 'roborun-sdk[urdf]'" in out.stdout
    assert "CAM numpy is not installed: pip install 'roborun-sdk[cameras]'" in out.stdout
    assert "LINK msgpack is not installed: pip install 'roborun-sdk[link]'" in out.stdout
    assert 'RC 1' in out.stdout and "roborun run: numpy is not installed: pip install 'roborun-sdk[robot]'" in out.stderr


def test_lazy_numpy_is_the_real_module_after_first_use():
    out = _python('from roborun_sdk import cameras\nimport numpy\n'
                  'assert type(cameras.np).__name__ == "lazy"\n'
                  'assert cameras.np.uint8 is numpy.uint8 and cameras.np is numpy\nprint("ok")')
    assert out.returncode == 0 and out.stdout.strip() == 'ok', out.stderr


# ------------------------------------------------------------------------------------------------ Windows
def test_importing_works_without_posix_only_modules():
    code = ('import importlib\n'
            f'for m in {LIGHT_MODULES!r}: importlib.import_module(m)\n'
            'from roborun_sdk import cli\n'
            'try: cli.main(["--help-all"])\n'
            'except SystemExit as e: assert not e.code\n'
            'from roborun_sdk.arms import canbus\nassert canbus.in_dialout() is False\nprint("ok")')
    out = _python(code, block=('fcntl', 'grp', 'pwd', 'termios', 'tty', 'pty', 'resource'), windows=True)
    assert out.returncode == 0, out.stderr
    assert out.stdout.strip().endswith('ok')


def test_arms_lock_uses_msvcrt_where_there_is_no_fcntl(tmp_path):
    code = (f'import os, sys, types\nos.environ["ROBORUN_HOME"] = {str(tmp_path)!r}\n'
            'calls = []\nm = types.ModuleType("msvcrt"); m.LK_NBLCK = 2\n'
            'def locking(fd, mode, n):\n'
            '    calls.append((mode, n))\n'
            '    if len(calls) > 1: raise OSError("locked")\n'
            'm.locking = locking; sys.modules["msvcrt"] = m\n'
            'from roborun_sdk.arms.safety import ArmsLock, ArmsBusy\n'
            'a = ArmsLock("first").acquire()\n'
            'try: ArmsLock("second").acquire(); print("not refused")\n'
            'except ArmsBusy as e: print("busy:", e)\n'
            'print(calls)')
    out = _python(code, block=('fcntl',), windows=True)
    assert out.returncode == 0, out.stderr
    assert 'busy: the arms are in use by first' in out.stdout and '[(2, 1), (2, 1)]' in out.stdout
