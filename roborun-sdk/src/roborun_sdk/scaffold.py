"""roborun new NAME: a small project for your own integration, outside Roborun.

    roborun new my-robot            # writes ./my-robot/
    roborun new my-robot --dir ~/code

The project holds everything a developer usually changes, as their own files: a driver for their arm (my_arm.py), rules
that can change or refuse any arm command (my_rules.py), a program that runs tasks over the local API (run_task.py), a
read-only check of the driver and the rules (check.py), the robot.yaml lines that use them, tests (tests/), a pyproject.toml
that pins the Roborun it was made with (roborun-sdk==<this version>), and a README. Nothing in Roborun itself is edited. See
docs/BUILDING.md for every extension point.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

from . import __version__
from .devkit import RELEASES_INDEX as DOWNLOADS_INDEX

NAME = re.compile(r'[A-Za-z][A-Za-z0-9_-]{0,63}')

MY_ARM = '''\
"""MyArm: the driver for your arm on Roborun.

Roborun calls this class; you never call Roborun. It subclasses StreamedArm, the same base the built-in drivers use, so you
get the parts that are the same for every arm without writing them: speed, acceleration and jerk limits on every target,
a smooth hold when commands stop, Stop braking, the temperature and stall guards, a dry run, and your rules file
(my_rules.py). You write the five *_hardware methods below for your motors.

Until you wire it, it drives SoftwareMotors, a stand-in that follows every command exactly, so the project runs as
generated. Replace SoftwareMotors with your motor bus (serial, CAN, your vendor's SDK); keep the method names.

In robot.yaml:
    arms:
      left:
        arm: {project}/my_arm.py:MyArm
        address: serial:///dev/ttyUSB0      # given to the driver as self.bus
"""
import time

from roborun_sdk.arms.base import ArmHardwareError, StreamedArm


class SoftwareMotors:
    """Stands in for a motor bus: six joints and a jaw that are exactly where they were last told to be."""

    def __init__(self, home):
        self.positions = list(home)
        self.jaw = 1.0                      # 0 closed .. 1 open
        self.enabled = False

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False

    def read(self):
        return list(self.positions), self.jaw

    def write(self, positions, jaw):
        if not self.enabled:
            raise ArmHardwareError('write to disabled motors')
        self.positions, self.jaw = list(positions), jaw


class MyArm(StreamedArm):
    # Joint angles in radians, base to wrist, one entry per arm joint. The arm profile Roborun plans for must have the
    # same number of joints (roborun profiles; roborun profiles create --urdf for your own).
    home = [0.0, -0.5, 1.0, 0.0, 0.5, 0.0]
    limits = [(-2.6, 2.6), (-1.6, 1.6), (-0.2, 2.6), (-1.8, 1.8), (-1.6, 1.6), (-2.8, 2.8)]
    rate_hz = 100.0                         # how often write_hardware() gets a new target
    joint_speed_rad_s = 0.6                 # every target is shaped to these; robot.yaml can lower or raise them
    joint_accel_rad_s2 = 2.0

    def open_hardware(self):
        """Open the bus and the motors without moving them. self.bus is the arm's `address:` from robot.yaml."""
        self.motors = SoftwareMotors(self.home)
        self.motors.enable()

    def open_dry(self):
        """For `dry_run: true` and check.py: open for reading only; enable and write nothing."""
        self.motors = SoftwareMotors(self.home)

    def read_hardware(self):
        """Where the arm is now. Called rate_hz times a second; raise if the bus fails, and Roborun stops the arm."""
        positions, jaw = self.motors.read()
        return dict(positions_rad=positions, gripper=jaw)
        # Optional keys: velocities_rad_s, temperatures_c (one per motor, jaw last), gripper_effort (0 free .. 1 at its limit)

    def write_hardware(self, positions_rad, gripper):
        """Send one target. Already within limits and shaped: send it as it is."""
        self.motors.write(positions_rad, gripper)

    def limp_hardware(self):
        """Stop holding: torque off, or gravity compensation only."""
        self.motors.disable()

    def close_hardware(self):
        self.motors.disable()

    def survey(self):
        """`roborun hw check`: read the arm without enabling or moving anything."""
        positions, jaw = SoftwareMotors(self.home).read()
        return dict(ok=True, positions_rad=positions, gripper=jaw, checked_at=time.time())
'''

MY_RULES = '''\
"""Your rules: Roborun calls check() for every arm command, before the arm's own limits and shaping.

    rules: {project}/my_rules.py        # in robot.yaml

Return False to refuse (the arm brakes and holds; a refusal that lasts ends the task with your rule's reason), the changed
(positions_rad, gripper) to adjust the command, or None to let it through. An exception also refuses: a broken rule never
lets motion through. Keep it fast: it runs up to the arm's rate (100 times a second).

state holds what the arm measured: positions_rad, velocities_rad_s, temperatures_c, gripper, gripper_effort, commanded_rad,
jaw_state, thermal, side, and tool_position_m / tool_pose (metres, arm base frame; None when the arm has no model).
"""

BASE_TURN_RAD = 2.0          # the base joint stays within +-2 rad: a keep-out behind the arm
MIN_TOOL_HEIGHT_M = 0.01     # refuse while the gripper is measured below the table surface (base frame z)


def check(side, positions_rad, gripper, state):
    tool = state.get('tool_position_m')
    if tool is not None and tool[2] < MIN_TOOL_HEIGHT_M:
        return False
    positions = list(positions_rad)
    positions[0] = max(-BASE_TURN_RAD, min(BASE_TURN_RAD, positions[0]))
    return positions, gripper
'''

RUN_TASK = '''\
"""Run a task on the robot on this computer and follow it until it ends.

    python run_task.py "pick up the cup"
    python run_task.py "pick up the cup" --mode preview      # plan it; the arms do not move

Uses the local API at http://127.0.0.1:8120 (this computer only), with nothing but the Python standard library, so any
Python 3 runs it. Ctrl-C stops the task. Exit code: 0 done, 1 failed or stopped, 2 the robot refused the request, 3 Roborun
is not running.
"""
import argparse
import json
import sys
import urllib.error
import urllib.request

API = 'http://127.0.0.1:8120'


class Refused(Exception):
    """The robot answered with an error: {"error": {"code", "message", "retryable"}}."""


def call(method, path, body=None, timeout=10):
    data = None if body is None else json.dumps(body).encode()
    request = urllib.request.Request(API + path, data=data, method=method, headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return json.load(response)
    except urllib.error.HTTPError as exc:
        raw = exc.read() or b'{}'
        try:
            error = json.loads(raw).get('error')
        except (ValueError, AttributeError):
            error = raw.decode('utf-8', 'replace')[:200]
        if isinstance(error, dict):                       # the API's errors: code, message, retryable
            error = f"{error.get('code')}: {error.get('message')}" + (' (try again)' if error.get('retryable') else '')
        raise Refused(f'{method} {path}: {exc.code} {error}') from None
    except (urllib.error.URLError, OSError) as exc:
        print(f'Roborun is not answering at {API} ({getattr(exc, "reason", exc)}): start it with roborun up', file=sys.stderr)
        raise SystemExit(3) from None


def follow(task_id):
    """The task's live stream: one JSON object each time something changes, until the task ends."""
    request = urllib.request.Request(f'{API}/v1/tasks/{task_id}/live', headers={'Accept': 'text/event-stream'})
    with urllib.request.urlopen(request, timeout=60) as stream:
        for line in stream:
            if line.startswith(b'data: '):
                yield json.loads(line[6:])


def main():
    global API
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('prompt')
    ap.add_argument('--mode', choices=('run', 'preview'), default='run')
    ap.add_argument('--arm', choices=('left', 'right'), help='which arm (default: Roborun chooses)')
    ap.add_argument('--api', default=API)
    a = ap.parse_args()
    API = a.api.rstrip('/')
    sys.stdout.reconfigure(line_buffering=True)          # each line as it happens, also into a file or a pipe

    status = call('GET', '/v1/status')
    print(f"robot: profile {status.get('profile')}, ready {status.get('ready')}")
    body = dict(prompt=a.prompt, mode=a.mode, **({'arm': a.arm} if a.arm else {}))
    try:
        task = call('POST', '/v1/tasks', body)
    except Refused as exc:
        print(exc, file=sys.stderr)
        return 2
    print(f"task {task['id']}: {task['status']}")
    last = None
    try:
        for view in follow(task['id']):
            seen = (view['status'], view.get('message'), view.get('moves'))
            if seen != last:
                print(f"{view['elapsed_s']:6.1f}s  {view['status']:8s} moves={view.get('moves')}  {view.get('message') or ''}")
                last = seen
            task = view
    except KeyboardInterrupt:
        print('stopping the task')
        task = call('POST', f"/v1/tasks/{task['id']}/stop", {})
    if task['status'] == 'running':                      # the stream closed early: read the task once more
        task = call('GET', f"/v1/tasks/{task['id']}")
    print(json.dumps({k: task.get(k) for k in ('id', 'status', 'moves', 'message', 'error', 'recording_url')}))
    return 0 if task['status'] == 'done' else 1


if __name__ == '__main__':
    sys.exit(main())
'''

CHECK = '''\
"""Check the driver and the rules the way Roborun loads them, without enabling or moving anything.

    roborun python check.py

(Roborun's own Python, where roborun_sdk is installed.) It opens my_arm.py:MyArm through Roborun's own-driver path in a dry run (open_dry: read only), with my_rules.py, sends one
command and prints what the arm would have been sent. Safe with your hardware connected.
"""
import sys
import time
from pathlib import Path

from roborun_sdk.arms import open_arm

HERE = Path(__file__).resolve().parent


def main(address=None):
    arm = open_arm(f'{HERE / "my_arm.py"}:MyArm', 'left', address, dry_run=True, rules=str(HERE / 'my_rules.py'))
    try:
        start = arm.read()
        print('read:', [round(v, 3) for v in start['positions_rad']], 'gripper', round(start['gripper'], 3))
        target = list(start['positions_rad'])
        target[0] = 3.0                                   # past the rules' base keep-out: my_rules.py trims it to 2.0
        arm.command(target, 0.5)
        after_rules = arm.held_target()                   # read at once: the command as your rules returned it, before shaping
        time.sleep(0.5)
        sent = arm.dry_log[-1] if arm.dry_log else None
        print('target after your rules:', [round(v, 3) for v in after_rules])
        print('would send now (shaped toward it):', sent)
        print('health:', {k: v for k, v in arm.health().items() if k in ('ok', 'error', 'dry_run', 'warnings')})
        return 0 if arm.health()['ok'] and sent else 1
    finally:
        arm.close()


if __name__ == '__main__':
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else None))
'''

ROBOT_YAML = '''\
# Add these lines to your robot.yaml (roborun settings --edit opens it), then run: roborun up
# Paths are absolute: robot.yaml lives in Roborun's settings folder, not in this project.
arms:
  left:
    arm: {project}/my_arm.py:MyArm
    address: serial:///dev/ttyUSB0      # your arm's bus; MyArm sees it as self.bus
    # joint_speed_rad_s: 0.4            # any other key reaches the driver (self.options) or its safety settings
rules: {project}/my_rules.py
'''

README = '''\
# {name}

A Roborun integration: your own arm driver, your own rules, and a program that runs tasks on the robot. Everything here is
yours to change; Roborun itself stays as installed.

| File | What it is |
| --- | --- |
| `my_arm.py` | The driver for your arm (`MyArm`). Runs as generated on software motors; replace `SoftwareMotors` with your bus. |
| `my_rules.py` | `check()` sees every arm command first and can change or refuse it. |
| `run_task.py` | Starts a task over the local API and follows it live until it ends. Standard library only. |
| `check.py` | Loads the driver and the rules the way Roborun does, in a dry run, and sends one command. Moves nothing. Run it with `roborun python check.py` (Roborun's own Python, where `roborun_sdk` is installed). |
| `robot.yaml` | The lines to add to your robot's settings so `roborun up` uses this driver and these rules. |

## Try it

```sh
roborun python check.py                           # the driver and the rules load; nothing is enabled
roborun up --sim cup                              # after `roborun login`: two simulated YAM arms
python3 run_task.py "pick up the cup"             # a task on the simulated arm, followed until it ends
```

`run_task.py` uses only the standard library, so any Python 3 runs it. For more, `roborun python` gives you
`roborun_sdk.client.Robot` (typed: tasks, Stop, camera frames, arm state); `roborun docs --examples ./examples` copies an LLM
tool layer built on it.

## Use your arm

1. Write your bus into `MyArm` (`open_hardware`, `read_hardware`, `write_hardware`, `limp_hardware`, `open_dry`), and set
   `home` and `limits` for your joints.
2. `roborun python check.py serial:///dev/ttyUSB0` reads your arm and shows what it would be sent, with nothing enabled.
3. Add the lines in `robot.yaml` to `roborun settings --edit`, with `dry_run: true` first if you want to watch the
   commands before anything moves, then `roborun up`.

Your rules run on drivers built on `StreamedArm` (like `MyArm` and the built-in ones). The simulator does not run them.

Stop: the Stop button in the app, `roborun stop`, or `curl -X POST http://127.0.0.1:8120/v1/stop` stops motion at any time.

## Tests

`tests/` checks your rules (no Roborun needed) and loads your driver through Roborun's own-driver path in a dry run (nothing is
enabled). Add a test for each rule you write.

```sh
roborun python -m pip install pytest       # once: pytest into Roborun's Python
roborun python -m pytest
```

## The Roborun version

`pyproject.toml` pins the Roborun this project was made with ({sdk_requirement}). In your own environment:

```sh
pip install --find-links {index} -e .      # or: uv sync (pyproject.toml names the same page)
```

To move to a newer Roborun: `roborun upgrade`, `roborun --version` for its version, then change the pin and run the tests.
Within a release series (0.1.x) the local API's `/v1` routes and fields and `roborun_sdk.client` stay compatible
(docs/BUILDING.md, "Versions and upgrades").
'''

PYPROJECT = '''\
# {name}: a Roborun integration. The dependency pins the Roborun it was made with; `roborun --version` shows the installed one.
[build-system]
requires = ["setuptools>=68"]
build-backend = "setuptools.build_meta"

[project]
name = "{name}"
version = "0.1.0"
requires-python = ">=3.10"
dependencies = ["{sdk_requirement}"]

[project.optional-dependencies]
test = ["pytest>=8"]

[tool.setuptools]
py-modules = []          # the files here run in place (roborun python check.py; robot.yaml names them by path)

[tool.pytest.ini_options]
testpaths = ["tests"]
pythonpath = ["."]

# Roborun's releases are not on PyPI: pip install --find-links {index} -e .
[tool.uv]
find-links = ["{index}"]
'''

TEST_RULES = '''\
"""my_rules.check(): the keep-out and the table. Plain Python: runs without Roborun installed."""
import my_rules

HOME = [0.0, -0.5, 1.0, 0.0, 0.5, 0.0]


def test_a_command_inside_the_rules_goes_through():
    assert my_rules.check('left', HOME, 1.0, dict(tool_position_m=[0.3, 0.0, 0.2])) == (HOME, 1.0)


def test_the_base_is_trimmed_to_the_keep_out():
    positions, gripper = my_rules.check('left', [3.0] + HOME[1:], 0.5, dict(tool_position_m=None))
    assert positions[0] == my_rules.BASE_TURN_RAD and gripper == 0.5


def test_below_the_table_is_refused():
    assert my_rules.check('left', HOME, 1.0, dict(tool_position_m=[0.3, 0.0, -0.05])) is False
'''

TEST_DRIVER = '''\
"""my_arm.MyArm through Roborun's own-driver path, in a dry run with my_rules.py: nothing is enabled or moved."""
import pytest

roborun_sdk = pytest.importorskip('roborun_sdk', reason='run with Roborun\\'s Python: roborun python -m pytest')

import check  # noqa: E402


def test_the_driver_and_the_rules_load_and_take_a_command():
    assert check.main() == 0
'''

FILES = {'my_arm.py': MY_ARM, 'my_rules.py': MY_RULES, 'run_task.py': RUN_TASK, 'check.py': CHECK,
         'robot.yaml': ROBOT_YAML, 'README.md': README, 'pyproject.toml': PYPROJECT,
         'tests/test_rules.py': TEST_RULES, 'tests/test_driver.py': TEST_DRIVER}


def sdk_requirement(version: str = __version__) -> str:
    """The installed Roborun, pinned: 'roborun-sdk==0.1.20261119172831' for a release; from a source checkout (0.1.0, never
    published) the series it belongs to, 'roborun-sdk>=0.1,<0.2'."""
    m = re.fullmatch(r'(\d+)\.(\d+)\.(\d+)', version)
    if m and m.group(3) == '0':
        major, minor = int(m.group(1)), int(m.group(2))
        return f'roborun-sdk>={major}.{minor},<{major}.{minor + 1}'
    return f'roborun-sdk=={version}'


def create(name: str, parent='.', force=False) -> Path:
    """Writes the project to <parent>/<name> and returns its folder."""
    if not NAME.fullmatch(name):
        raise SystemExit(f'{name!r}: use letters, digits, - and _ (starting with a letter)')
    root = (Path(parent).expanduser() / name).resolve()
    if root.exists() and any(root.iterdir()) and not force:
        raise SystemExit(f'{root} already exists and is not empty (--force writes over the generated files)')
    root.mkdir(parents=True, exist_ok=True)
    fill = {'{project}': str(root), '{name}': name, '{sdk_requirement}': sdk_requirement(), '{index}': DOWNLOADS_INDEX}
    for file, text in FILES.items():
        for key, value in fill.items():
            text = text.replace(key, value)
        (root / file).parent.mkdir(parents=True, exist_ok=True)
        (root / file).write_text(text)
    return root


def cmd_new(a) -> int:
    root = create(a.name, a.dir, a.force)
    print(f'created {root}')
    for file in FILES:
        print(f'  {file}')
    print(f'pinned: {sdk_requirement()} (pyproject.toml)')
    print(f'next: cd {root} && roborun python check.py, then add robot.yaml to `roborun settings --edit` (see README.md)')
    return 0


def add_parser(sub):
    p = sub.add_parser('new', help='start your own integration: a project with your arm driver, rules and a task client')
    p.add_argument('name', help='the project folder to create')
    p.add_argument('--dir', default='.', help='where to create it (default: here)')
    p.add_argument('--force', action='store_true', help='write over the generated files in an existing folder')
    p.set_defaults(func=cmd_new)


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser(prog='roborun new')
    add_parser(ap.add_subparsers(dest='cmd', required=True))
    args = ap.parse_args(['new', *sys.argv[1:]])
    sys.exit(args.func(args))
