# Building on Roborun

This is the Roborun reference: installing and running it, your arms, profiles and simulation, privacy, and every way to
build on it. The [README](../README.md) is the quickstart; [the hosted API appendix](HOSTED_API.md) covers the service
behind it.

Roborun is made to be extended from your own files. You never edit Roborun: you point `robot.yaml` at your code, or you
talk to the robot's local API from your own program. Updates to Roborun then never overwrite your work, and your work
never weakens the safety parts every robot shares.

This guide maps every extension point, with a small example for each. The task, driver, camera and rules outputs shown
are from runs against a real Roborun install (a simulated arm where no hardware was needed); the shapes of the newer Stop,
camera and client calls are as the local API defines them (`GET /v1/openapi.json`).

An AI assistant running Roborun for someone should read `roborun docs agents` first: one short file with the commands,
routes, errors and what to do next, linking here.

The guide, the README, the hosted API notes and the examples ship with Roborun: `roborun docs` says where they are,
`roborun docs building` prints this guide, and `roborun docs --examples DIR` copies the examples into DIR. Run your Python
with Roborun's own interpreter, where `roborun_sdk` is installed: `roborun python my_script.py`.

## How the pieces fit

```
 your computer (wired to the arms)                                              hosted
 ┌──────────────────────────────────────────────────────────────────────┐
 │  your arm driver ─┐                                                  │
 │  your rules ──────┼─ station: shaping, guards, Stop, Home   (arms)   │
 │  catalog drivers ─┘                                                  │
 │  your camera ──── camera service: capture, face blur                 │
 │                                                                      │   motion
 │  robot runtime ── local API http://127.0.0.1:8120 ───────────────────┼─▶ https://api.roborun.app
 │       ▲                ▲                                             │
 │       │                └── your app or agent                         │
 │  page connector 127.0.0.1:8135 ◀── https://app.roborun.app (browser  │
 │                                     on this computer)                │
 └──────────────────────────────────────────────────────────────────────┘
```

The hosted API turns a prompt and the camera view into motion. Everything that touches the arms runs on your computer:
the drivers, the speed and jerk limits, the guards, your rules, and Stop.

| You want to | Extension point | Where it goes |
| --- | --- | --- |
| Drive an arm Roborun does not know | [Your own arm driver](#your-own-arm-driver) | `arms: left: arm: /path/my_arm.py:MyArm` |
| Use any image source | [Your own camera](#your-own-camera) | `cameras: top: source: /path/my_cam.py:MyCamera` |
| Keep-out zones, extra limits, interlocks | [Rules](#rules) | `rules: /path/my_rules.py` |
| Plan for your arm's shape | [Arm profile from a URDF](#arm-profile-from-a-urdf) | `roborun profiles create --urdf ...` |
| Run tasks from an app, a script or an agent | [The local API](#the-local-api) | `http://127.0.0.1:8120/v1/...` |
| Do it from Python | [The Python client](#the-python-client) | `from roborun_sdk.client import Robot` |
| Give the robot to an LLM | [An LLM's tools](#an-llms-tools) | `examples/llm_tools.py` |
| Pin the Roborun your code was built on | [Versions and upgrades](#versions-and-upgrades) | `roborun-sdk==0.1.<time>` |

## Install, upgrade and remove

```sh
curl -fsSL https://roborun.app/install.sh | sh
```

It works on macOS and Linux (Windows: WSL2 with Ubuntu), installs without administrator rights into `~/.roborun` (and
`~/.local/bin/roborun`; open a new terminal or `export PATH="$HOME/.local/bin:$PATH"`), and brings its own Python. Two things
may need one administrator command, and Roborun prints the exact line when they are missing: the simulator needs an OpenGL
library (`sudo apt install libosmesa6` on Ubuntu/Debian, `sudo dnf install mesa-libOSMesa` on Fedora), and USB arms need the
udev and `dialout` steps `roborun setup` shows.

| Do | Command | You see |
| --- | --- | --- |
| Check | `roborun --version` | `roborun-sdk 0.1.<time> (build <id>)`, the robot runtime, Python |
| Upgrade | `roborun upgrade` | `already on the newest Roborun (...)`, or `upgraded (<build>)` and a restart (never while a task runs) |
| Remove | `roborun uninstall` | It stops Roborun (no arm is dropped) and removes `~/.roborun`, the `roborun` command and the PATH line the installer added. It asks before deleting your settings, key and recordings (`~/.config/roborun`, `~/roborun`): `--purge` deletes them, `--keep-data` keeps them, `--yes` does not ask |

If the installer says no release is published yet, a build id you were given installs with `ROBORUN_BUILD=ID`; a release
installs with `ROBORUN_VERSION=<version>` ([Versions and upgrades](#versions-and-upgrades)).

Files: your settings and key in `~/.config/roborun/` (`token`, `robot.yaml`, which is yours to edit, and `robot.json`, which is
generated); the robot runtime, its logs and recordings in `~/roborun/robot/runtime/`; datasets in `~/roborun/datasets/<robot>/`.

## Sign in and keys

Every robot needs a Roborun key (`rr_live_...`, or `rr_test_...`). Roborun is invite-only: if you don't have one, email
support@roborun.dev.

- `roborun login` prints `Open https://app.roborun.app/device?code=XXXX-XXXX and approve this computer.` and exits. Once the
  link is approved, the next `roborun up` saves a new key in `~/.config/roborun/token` (and waits for the approval if it has
  not happened yet). `roborun login --wait` waits for the approval right there instead.
- A key you already have: save it in a file, `chmod 600 FILE`, and pass `--key-file FILE` to `roborun up` once; or set
  `ROBORUN_API_KEY`.

`roborun status` shows `motion service ... (token accepted)` when the key works. A revoked key: `roborun login` again.

## Running Roborun

| Do | Command |
| --- | --- |
| Start with simulated arms (no hardware) | `roborun up --sim cup` |
| Start with your arms | `roborun up` (the first time: the guided setup, which needs a terminal) |
| Back to your arms after a simulation | `roborun up --no-sim` |
| What runs, and is it healthy | `roborun status` |
| Stop motion now | `roborun stop` (also the Stop button and `POST /v1/stop`) |
| Stop Roborun | `roborun down` |
| This robot's own viewer | `roborun view` |

`roborun up` needs nothing else after `roborun login`: the robot is named after this computer (`--robot NAME`, or `robot:`
in `robot.yaml`), the key is the one `roborun login` saved, and the profile is the arms you set up (one YAM arm, `yam`, when
nothing names them; [Profiles and simulation](#profiles-and-simulation)). Without a terminal and with no arms set up, it
starts nothing and prints the manual steps ([Your arms](#your-arms-setup-and-hardware-checks)).

A good start ends with `ready: open https://app.roborun.app on this computer, or this robot's own viewer:
http://127.0.0.1:8135/` and `local API: http://127.0.0.1:8120/v1/docs`. The first start downloads the robot runtime (about a
minute) and sets up recording once (about 290 MB: pyarrow and PyAV, for LeRobot-format datasets); `roborun up --no-dataset`
skips that (remembered). If that setup fails, the robot still runs, and each recording says `recorded: false` with the reason.

`roborun down` stops everything and drops no arm: an arm that is not resting on something stays powered and held, and it
says so. Then `roborun down --park` moves it slowly to its rest pose and switches the motors off, or `roborun down --release`
switches them off where they are while you support the arm.

- Under systemd the robot stops when you log out unless "linger" is on. `roborun up` turns it on where it can (a desktop
  session); otherwise it prints the one admin command, `sudo loginctl enable-linger $USER`. `roborun status` says
  `after logout: keeps running` or `stops`. `ROBORUN_NO_LINGER=1` leaves it alone.
- Logs: `journalctl --user -u roborun-robot` (and `roborun-sim`, `roborun-station`, `roborun-local`, `roborun-cameras`);
  without systemd, in `~/.config/roborun/logs/`.
- `port 8120 ... in use by another program`: stop that program (`lsof -i :8120`), then `roborun up` again.

## Your arms: setup and hardware checks

Plug in your arms and cameras, power the arms, and run `roborun up` (or `roborun setup`, any time later). The guided setup:

1. finds your CAN and serial adapters and your cameras, and says what to check if it finds none;
2. asks the motors what they are, without enabling anything, and suggests the arms they could belong to. With two arms it
   helps you tell the adapters apart by unplugging one;
3. assigns your cameras to top, left and right;
4. reads each arm with nothing enabled, and catches a wrong choice there;
5. runs the calibration your arm needs (keep the arm supported and the jaws empty);
6. offers the movement check: small, slow moves, one joint at a time. If you skip it, that is recorded and `roborun up`
   keeps reminding you;
7. writes `robot.yaml`, with comments, and starts. Then it shows the overhead camera's status (below).

Ctrl-C is safe at any point: no motor stays on, your answers are kept, and `roborun setup` continues from there. Your own
edits to `robot.yaml` are kept. Every step also runs on its own, which is what a person (or an assistant) without the guided
setup uses:

```sh
roborun hw detect                                 # adapters (with their ADDRESS), cameras, and which arms they could be; asks and moves nothing
roborun hw arms                                   # every arm in the catalog, with its status
roborun hw check yam@slcan://SERIAL               # reads the arm; enables nothing
roborun hw calibrate so101@serial://SERIAL        # by-hand calibration, for arms whose check asks for it
roborun hw calibrate --gripper yam@slcan://SERIAL # YAM: the jaw finds its two ends (jaws empty; only the jaw moves)
roborun hw check yam@slcan://SERIAL --move        # tiny guided movement, one joint at a time (needs a terminal)
roborun hw motor list damiao@can0                 # the motors on a bus; also set-id and set-zero, each asks first
```

```yaml
# ~/.config/roborun/robot.yaml
arms:
  left:
    arm: yam
    address: slcan://SERIAL
cameras:
  top: realsense:SERIAL
  left: realsense:SERIAL
# dry_run: true       # read the arms and log what would be sent: nothing is enabled or written
```

The catalog (`roborun hw arms`) lists 35 arms: 32 supported in beta (each `beta` until we have proven it on real hardware;
the setup says so when you pick one) and 3 bring-your-own entries, where you supply [the driver](#your-own-arm-driver).

**The overhead camera finds its own place.** There is nothing to print or place. After the arms and cameras connect, the
robot measures the table in the overhead depth image and finds its arm there; `roborun up` and `roborun status` show the
result. With two arms in view that is enough. With one arm, one pose cannot prove that what the camera found is the arm:
previews work, and the arm will not move until it has been seen in a second pose (press Home in the app, or move the arm by
hand). If the camera or an arm base is moved, the robot finds the new pose. The overhead camera must give depth (an Intel
RealSense, for example) and must see the table and the arm's gripper.

## Profiles and simulation

A profile is the arm shape Manifest plans for. `roborun profiles` lists them (`*` = running); switch with
`roborun up --profile NAME`, or `profile:` in `robot.yaml`. Your arms usually choose it for you.

| Profile | What it is |
| --- | --- |
| `yam` | One YAM arm |
| `yam_dual` | Two YAM arms |
| `yam_metal` | A YAM arm and a MakerMods Metal arm |
| `<arm>`, `<arm>_dual`, `<left>+<right>` | Any catalog arm: one, two of the same, or two different ones (`so101`, `so101_dual`, `so101+koch_v1_1`) |
| your own | [Arm profile from a URDF](#arm-profile-from-a-urdf): `roborun profiles create --urdf ...` |
| `booster_k1`, `galaxea_r1` | Humanoids ([Humanoids](#humanoids-booster-k1-and-galaxea-r1)) |

`yam_single` and `makerarm` were renamed (to `yam` and `maker_arm_dual`); the old names still work for a while and say so.

`roborun up --sim SCENE` simulates the profile on this computer instead of real arms; with no profile it runs two YAM arms
(`yam_dual`), the scene every task is proven on. Arm scenes: `cup` (one cup, the default), `pickup-bench` (two boxes),
`mixed-objects` (a pack, a bottle, a mug and a hammer), `cluttered-bench` (two boxes among smaller objects) and `low-light`
(the pickup bench, dimly lit). Humanoid scenes: `room`, `empty`. `sim: cup` in `robot.yaml` does the same; the simulator
never starts on its own, and `roborun up --no-sim` goes back to your arms.

## Privacy and your data

Camera images stay sharp on this computer: the robot sees them that way, and so do your recordings and the live view. Every
image or depth frame that leaves this computer has faces blurred first, as a best effort: frames sent for motion, and your
history. If the blur cannot run, nothing is sent, and the robot can still see and stop. `privacy: {face_blur: false}` in
`robot.yaml` sends frames for motion unblurred and keeps history images and depth on this computer (task records are still
uploaded).

**One exception, for now: a humanoid's head frame is not blurred yet.** On a humanoid profile (`booster_k1`,
`galaxea_r1`), the head camera frame that each move sends to the hosted API goes as it is, and `GET /v1/cameras/head.jpg`
(also `/v1/body/frame.jpg`) and `Robot.frame('head')` give the sharp frame. Keep people out of the head camera's view
during moves, and don't send those frames on. The head frames in your history are blurred on upload like every other image.

Your task records, recordings and the robot's telemetry are uploaded to your Roborun account so you can see your history:
the telemetry is one file per hour with the arms' joint positions, jaws, speeds and motor temperatures once a second, task or
not, and events such as Home or a fault (no images). `roborun up --no-data-sync` turns all of that off (`--data-sync` on again). Each robot's history is also a LeRobot v2.1 dataset, read with your key
(`Authorization: Bearer KEY`; `{robot}` is the name `roborun status` shows):

| Call | Answers |
| --- | --- |
| `GET https://api.roborun.app/v1/robots/{robot}/episodes` | The dataset (version, robot type, fps, totals) and its episodes: prompt, length, outcome, when |
| `GET https://api.roborun.app/v1/robots/{robot}/dataset.tar` | The whole dataset; unpack it and load `dataset/` with LeRobot. Faces are blurred |
| `POST https://api.roborun.app/v1/robots/{robot}/data/delete` | With `{"confirm": "<robot>"}`: deletes everything kept for that robot. It cannot be undone |

## The viewer and the page

`roborun view` (or <http://127.0.0.1:8135/> while Roborun runs) opens this robot's own viewer: its status, live cameras,
recent tasks with their outcome, recordings played from the robot's own cameras, and Stop. It needs no sign-in, works
offline, and only reads. <https://app.roborun.app> is Roborun's page: open it on the robot's computer (it talks to the robot
only through `127.0.0.1:8135`). Arm profiles get the arm page (prompt, cameras, 3-D arms, Stop, Home); humanoid profiles get
the roving page.

## Start a project

```sh
roborun new my-robot
```

```
created ~/my-robot
  my_arm.py
  my_rules.py
  run_task.py
  check.py
  robot.yaml
  README.md
  pyproject.toml
  tests/test_rules.py
  tests/test_driver.py
pinned: roborun-sdk==0.1.20261119172831 (pyproject.toml)
next: cd ~/my-robot && roborun python check.py, then add robot.yaml to `roborun settings --edit` (see README.md)
```

| File | What it is |
| --- | --- |
| `my_arm.py` | A driver for your arm, built on `StreamedArm`. It runs as generated, on software motors; you replace those with your bus. |
| `my_rules.py` | `check()`, called for every arm command: keeps the base inside +-2 rad and refuses while the gripper is below the table. |
| `run_task.py` | Starts a task over the local API and follows it live until it ends. Standard library only: any `python3` runs it. |
| `check.py` | Loads the driver and the rules exactly as Roborun does, in a dry run (nothing enabled), and sends one command. Run it with `roborun python check.py`. |
| `robot.yaml` | The lines that make `roborun up` use this driver and these rules, with absolute paths filled in. |
| `pyproject.toml` | Pins the Roborun the project was made with (`roborun-sdk==<roborun --version>`); see [Versions and upgrades](#versions-and-upgrades). |
| `tests/` | `test_rules.py` checks the rules (plain Python); `test_driver.py` runs `check.py`'s dry run. `roborun python -m pytest` runs both (once: `roborun python -m pip install pytest`). |

`check.py` is safe with your hardware connected:

```
$ roborun python check.py
read: [0.0, -0.5, 1.0, 0.0, 0.5, 0.0] gripper 1.0
target after your rules: [2.0, -0.5, 1.0, 0.0, 0.5, 0.0]
would send now (shaped toward it): {'positions_rad': [0.186, -0.5, 1.0, 0.0, 0.5, 0.0], 'gripper': 0.7}
health: {'ok': True, 'error': None, 'dry_run': True, 'warnings': []}
```

The command asked for 3.0 rad on the base; the rules trimmed it to 2.0, and the stream shapes the motion toward it at the
arm's speed and acceleration limits.

With a simulated robot running, `run_task.py` goes through the whole path, the hosted motion included:

```sh
roborun up --sim cup                 # after `roborun login`
python3 run_task.py "pick up the cup"
```

```
task 0123456789abcdef0123456789abcdef: running
   0.1s  running  moves=0  Checking the overhead camera…
  15.6s  running  moves=0  Planning the motion…
  20.9s  running  moves=0  Moving: reach, close, lift…
  48.5s  done     moves=8  Finished: 8 moves
{"id": "0123456789abcdef0123456789abcdef", "status": "done", "moves": 8, "message": "Finished: 8 moves", "error": null,
 "recording_url": "/v1/tasks/0123456789abcdef0123456789abcdef/recording"}
```

## Your own arm driver

A driver is a Python class in your own file. Name it in `robot.yaml`, or with `--left` / `--right`:

```yaml
arms:
  left:
    arm: ~/my-robot/my_arm.py:MyArm     # or package.module:MyArm, if it is installed
    address: serial:///dev/ttyUSB0              # given to the driver as `bus`
    joint_speed_rad_s: 0.4                      # any other key: a safety setting, or passed to the driver
```

```sh
roborun up --left ~/my-robot/my_arm.py:MyArm@serial:///dev/ttyUSB0
```

There are two ways to write one. Use `StreamedArm` unless you have a reason not to: it is the one stable driver base.

**Built on `StreamedArm` (recommended).** This is how every built-in driver is written, and what `roborun new` generates.
You implement five small methods; Roborun runs a 100 Hz stream around them that limits speed, acceleration and jerk on
every target, brakes smoothly on Stop, holds when commands stop arriving, watches temperatures, blocked joints and the jaw,
supports `dry_run`, and applies your rules file.

```python
from roborun_sdk.arms.base import StreamedArm

class MyArm(StreamedArm):
    home = [0.0, -0.5, 1.0, 0.0, 0.5, 0.0]                     # rad, one per arm joint
    limits = [(-2.6, 2.6), (-1.6, 1.6), (-0.2, 2.6), (-1.8, 1.8), (-1.6, 1.6), (-2.8, 2.8)]

    def open_hardware(self):        # open the bus (self.bus) and motors without moving them
        self.motors = MyBus(self.bus); self.motors.enable()
    def open_dry(self):             # for dry_run: read only, enable and write nothing
        self.motors = MyBus(self.bus)
    def read_hardware(self):        # where the arm is; raise on a bus failure and Roborun stops the arm
        q, jaw = self.motors.read()
        return dict(positions_rad=q, gripper=jaw)               # gripper: 0 closed .. 1 open
    def write_hardware(self, positions_rad, gripper):           # already limited and shaped: send as is
        self.motors.write(positions_rad, gripper)
    def limp_hardware(self):        # stop holding: torque off or gravity compensation
        self.motors.disable()
```

`read_hardware()` may also return `velocities_rad_s`, `temperatures_c` (one per motor, the jaw last) and `gripper_effort`
(0 free .. 1 at its force limit). With `gripper_effort`, Roborun can tell a jaw that closed on an object from an empty one.
Optional methods: `close_hardware()`, `survey()` (the read-only check behind `roborun hw check`), `thermal_motors()`,
`motors_off_hardware()` with `rest_pose`. Settings from `robot.yaml` that Roborun does not know are in `self.options`.

**A plain `ArmAdapter` (the low-level path).** The smallest driver: `read()` and `command()`. The station sends a smooth, step-limited target
at 30 Hz, and Stop holds the last measured position. There is no 100 Hz shaping, no guards and no rules file. See
[examples/minimal_arm.py](../examples/minimal_arm.py):

```python
from roborun_sdk.embodiment import ArmAdapter

class MinimalArm(ArmAdapter):
    home = [0.0, -0.5, 1.0, 0.0, 0.5, 0.0]
    limits = [(-2.6, 2.6)] * 6
    def __init__(self, bus=None): ...                     # takes only the settings its __init__ names
    def read(self):                                       # {positions_rad, gripper}
        ...
    def command(self, positions_rad, gripper):
        ...
```

```
$ python examples/minimal_arm.py
{"event": "station", "url": "http://127.0.0.1:38129", "left": "MinimalArm", "right": "parked"}
{"station": "http://127.0.0.1:38129", "left": {"schema": "manifest-native-pose/1", ... "positions_rad": [0.0, -0.5, 1.0, 0.0, 0.5, 0.0], "gripper": 1.0, ...
```

Either way:

- The number of joints must match the profile Roborun plans for (`roborun profiles`); `roborun up` refuses a mismatch.
  For a new arm shape, make a profile from its URDF (below).
- Check it before anything moves: `roborun python check.py` in a generated project, then `dry_run: true` in `robot.yaml` to run
  `roborun up` with every command logged instead of sent, then `roborun hw check ARM@ADDRESS --move` for a guided,
  one-joint-at-a-time movement check.
- To test the whole path before your hardware is wired, `roborun_sdk.embodiment:KinematicArm` is a software arm.

The generated driver running under `roborun up` (from the run behind this guide; the station reports `MyArm` on the left):

```
$ roborun up --no-sim
arms: left ~/my-robot/my_arm.py:MyArm
robot: 127.0.0.1:8120, profile yam
ready: open https://app.roborun.app on this computer, or use the API at http://127.0.0.1:8120
note: no cameras are set up, so tasks cannot plan. Add them with: roborun setup
```

## Your own camera

Any class with `read()` returning an RGB image (`uint8`, height x width x 3), or `None` when no new frame is ready:

```yaml
cameras:
  top:
    source: ~/my-robot/folder_camera.py:FolderCamera
    folder: ~/frames          # every other key is passed to __init__
    fps: 10
```

```python
class FolderCamera:
    def __init__(self, folder, role='top', fps=10): ...     # gets whichever of role, width, height, fps it names
    def open(self): return dict(serial='folder:frames')     # optional: serial, intrinsics, depth_scale_m
    def read(self): ...                                     # RGB image, or {rgb, depth_z16}, or None
```

Return `{rgb, depth_z16}` from `read()` and `depth_scale_m` from `open()` to give depth; the overhead camera needs depth
and its intrinsics. Optional: `close()`, `set_exposure_us(us)`. Faces are blurred on every frame that leaves the computer,
whatever the source, except a humanoid's head frame, not blurred yet ([Privacy](#privacy-and-your-data)). Full example:
[examples/folder_camera.py](../examples/folder_camera.py), opened the way Roborun opens it:

```
$ python examples/folder_camera.py ~/frames
opened: {'serial': 'folder:frames'}
frame 1 (480, 640, 3) uint8
frame 2 (480, 640, 3) uint8
frame 3 (480, 640, 3) uint8
```

## Rules

A rules file can change or refuse every command, on every arm built on `StreamedArm` (all built-in drivers and the
generated `MyArm`):

```yaml
rules: ~/my-robot/my_rules.py
```

```python
def check(side, positions_rad, gripper, state):
    tool = state.get('tool_position_m')                 # metres, arm base frame; None without the arm's model
    if tool is not None and tool[2] < 0.01:
        return False                                    # refuse: the arm brakes and holds
    positions = list(positions_rad)
    positions[0] = max(-2.0, min(2.0, positions[0]))    # change: keep the base inside +-2 rad
    return positions, gripper                           # or None: let it through
```

- `state` holds what the arm measured: `positions_rad`, `velocities_rad_s`, `temperatures_c`, `gripper`,
  `gripper_effort`, `commanded_rad`, `jaw_state`, `thermal`, `side`, `tool_pose` and `tool_position_m`.
- A refusal that lasts (0.4 s) ends the task with "your rules file refused this motion". An exception in `check()` also
  refuses: a broken rule never lets motion through.
- It runs at the arm's stream rate (100 times a second): keep it fast and free of I/O.
- Your limits come on top of Roborun's: the arm's joint limits and shaping still apply to what you return.
- Rules apply to real arms. The simulator and plain `ArmAdapter` drivers do not run them.

[examples/keep_out_box.py](../examples/keep_out_box.py) keeps the gripper out of a box on the table:

```
$ python examples/keep_out_box.py
gripper at (0.40, 0.00, 0.10): False
gripper at (0.20, 0.25, 0.10): None
```

## Arm profile from a URDF

A profile tells the motion service your arm's shape: its joints, gripper, grasp point and where it sits on the table.
The catalog arms (`roborun hw arms`) have profiles already. For your own arm:

```sh
roborun profiles create --name my_arm --urdf my_arm.urdf \
  --joints link_002_joint,link_003_joint,link_004_joint,link_005_joint,link_006_joint,link_007_joint \
  --fingers gripper_left_joint,gripper_right_joint --tcp-link grasp_center --base-link base_link --first-link link_002
```

```
profile my_arm: 1 arm(s), model ~/roborun/robot/runtime/models/custom/my_arm/my_arm.xml
  reach 0.715 m, jaw stroke 52.4 mm, IK check 0.01 mm
  next: try it in simulation: roborun up --profile my_arm --sim cup; for your hardware, write its driver on StreamedArm (roborun new my-robot), then: roborun up --profile my_arm
```

The profile is written into the robot runtime's folder (`~/roborun/robot/runtime/profiles/my_arm` and `models/custom/my_arm`).
Keep your URDF and the `roborun profiles create` command in your own project, so you can make it again on another computer.

Then `roborun up --profile my_arm --sim cup` runs it in simulation, and `roborun up --profile my_arm` with your driver
runs it for real. `--fingers` also takes one revolute jaw joint (`--jaw-closed-rad`, `--jaw-open-rad`); `--arms 2` and
`--place left:x,y` set up two arms; `roborun profiles create --help` lists the rest. MuJoCo MJCF files work too.

## The local API

The robot answers at `http://127.0.0.1:8120`, on this computer only. Use it from any language; it needs no key, because
only programs on this computer can reach it. Web pages cannot call it (they go through the page connector).

| Call | What it does |
| --- | --- |
| `POST /v1/tasks` | Start a task: `prompt` (1-1500 characters), optional `mode` (`run` moves the arms, `preview` does not), `arm` (`left`, `right`, `both`; default: chosen from the prompt and the scene). `model` may be left out: there is one model, Manifest (`manifest-1-beta`) |
| `GET /v1/tasks`, `GET /v1/tasks/{id}` | Recent tasks (`?limit=1..200`); one task |
| `GET /v1/tasks/{id}/live` | The task and its current motion; with `Accept: text/event-stream`, a stream until it ends |
| `GET /v1/tasks/{id}/recording` | What the robot's cameras recorded during the task |
| `POST /v1/tasks/{id}/stop` | Stop this task |
| `POST /v1/stop`, `GET /v1/stop/status` | Stop all motion, like the Stop button; whether everything has stopped |
| `GET /v1/cameras/{role}.jpg` | The newest frame of the `top`, `left` or `right` camera (JPEG), faces blurred |
| `GET /v1/robot` | The arms now: `joints_rad`, `gripper` (0 closed .. 1 open), `tool_mm` per arm, cameras, safety |
| `GET /v1/status`, `GET /v1/health` | Profile, arms, cameras and the running task; is it up |
| `GET /v1/docs`, `GET /v1/openapi.json` | This list, for people and for code generators |

`/v1` is stable within this version: routes and fields are added, never renamed or removed. Other routes on this port
(`/api/...`, `/internal/...`) belong to the page and change without notice; `POST /api/stop` and `GET /api/stop/status` keep
working for programs that already use them.

A task's `status` is `running`, `done`, `stopped` or `failed`. A finished task says what happened in `message` and how many
moves it made in `moves`, and what it achieved in `outcome`, read from the checks the task itself ran:

```json
"outcome": {
  "holding": {"left": {"object": "cup", "lifted_mm": 91, "evidence": "grip+lift", "verified": true}, "right": "unknown"},
  "placed": [],
  "verified_by": ["grip", "lift"]
}
```

- `holding.left` / `holding.right`: what that arm holds now that the task has ended. An object (`object`, `lifted_mm` above
  where it was grasped, `evidence`, `verified`: every check the task runs for that step passed); `null` when the task
  measured it empty (the jaw open or closed on nothing, the object let go, or a camera saw the object stay behind); or
  `"unknown"` when the task ran no check on that arm. `"unknown"` is never a guess either way: look before you rely on it.
- `evidence` and `verified_by` name the checks: `grip` (the jaw closed on something: its contact, its grip force, or a
  verified hold it remembers), `lift` (measured at least 20 mm above where it closed), `wrist_camera` (a wrist camera saw the
  object rise with the jaw), `overhead_camera` (the overhead depth camera saw it rise), `release` (the jaw opened to let go).
- `placed`: what the task let go of, `[{arm, object, destination, distance_mm, verified}]`; `"unknown"` when the task has no
  checks to read.
- `outcome` is `null` while the task runs, for a preview and when nothing moved.

After "pick up the cup" a program checks `task['outcome']['holding']['left']`; after "put it down" that is `null`. Do not
judge a hold from `gripper` in `GET /v1/robot`: a jaw around a wide cup reads close to open (one read 0.947 while holding a cup).

### Errors

Every error on `/v1` has one shape:

```json
{"error": {"code": "busy", "message": "task 8b22... is still running; stop it first", "retryable": true}}
```

`message` is plain and fine to show to a person or an LLM. `retryable` says whether the same request may work later
or must change:

| `code` | `retryable` | What happened |
| --- | --- | --- |
| `invalid` | no | The request is wrong: an empty prompt, an unknown field, a body that is not JSON |
| `not_found` | no | No such task, camera or route |
| `method_not_allowed` | no | That route does not take this method |
| `forbidden` | no | A route that belongs to the page (Home, settings from the page); use the `/v1` routes |
| `busy` | yes | Another task is running: wait for it, or stop it |
| `unavailable` | yes | A camera has no fresh frame yet |
| `blur_unavailable` | yes | Face blur could not run, so no frame is given |
| `stop_not_confirmed` | yes | A Stop could not be delivered or confirmed (`503`): support the arm or cut its power |
| `unreachable` | yes | The arms do not answer (`GET /v1/stop/status`, `503`) |
| `internal` | no | A bug in the robot runtime: its log says more (`journalctl --user -u roborun-robot`) |

The Python client adds `not_running` (status 0) when nothing answers at all. Stop is never refused.

### Following a task

The live stream (`Accept: text/event-stream`) sends one `data: {task}` event each time the task changes, and at least every
5 s while nothing changes; the last event carries the final status, then the stream closes. Polling `GET /v1/tasks/{id}`
works too.

```python
import json, urllib.request
request = urllib.request.Request(f'http://127.0.0.1:8120/v1/tasks/{task_id}/live', headers={'Accept': 'text/event-stream'})
with urllib.request.urlopen(request, timeout=60) as stream:
    for line in stream:
        if line.startswith(b'data: '):
            view = json.loads(line[6:])
            print(view['status'], view['message'])
```

A recording lists each camera's frame count; frames are numbered from 0, so the last one is the count minus one
(`last_frame_url` names it):

```json
{"task": "7551b76f...", "recordings": [{"name": "recording-...", "hz": 10, "cameras": {"top": 420, "left": 419},
  "frame_url": "/v1/video-motion/.../{camera}/{frame}.jpg", "last_frame_url": {"top": ".../top/419.jpg", "left": ".../left/418.jpg"}}]}
```

### Stop

`POST /v1/stop` stops all motion, like the Stop button. It is never refused, takes no body, and answers small:

```json
{"state": "stopped", "confirmed": true, "stopped_tasks": ["694acbc5..."], "arms": {"left": "holding", "right": "parked"},
 "message": "Stopped: the arms are holding"}
```

`200` means the Stop was taken: `state` is `stopped` (the arms hold) or `stopping` (an arm still brakes). `503` means it could
not be delivered or confirmed (`state` `not_confirmed`, with an `error`): support the arm or cut its power. `?verbose=1` adds
the station's full answer as `detail`. `GET /v1/stop/status` reads the arms twice and says whether anything still moves:

```json
{"state": "stopped", "confirmed": true, "arms": {"left": {"state": "still", "max_speed_rad_s": 0.0}},
 "task": null, "max_speed_rad_s": 0.0, "message": "Nothing is moving: the arms are holding still"}
```

`state` is `stopped`, `stopping` (a task is ending or an arm brakes), `running` (a task runs) or `moving`. A Stop does not
latch: the next task you start runs. After a Stop the arms hold where they are, and the next task starts from there. An
agent that drives the robot should treat Stop as always available and call it on any doubt.

### Cameras

`GET /v1/cameras/top.jpg` (or `left`, `right`) is the newest frame. Faces are blurred, with the same blur as every frame that
leaves this computer, so you may send it on, to an LLM for example. `?blur=0` gives the sharp frame, for use on this computer
only; if the robot's owner turned face blur off (`privacy: {face_blur: false}`), frames come sharp. `503` with `retryable`
means no fresh frame yet; `503` `blur_unavailable` means the blur could not run, and then no frame is given.

## The Python client

`roborun_sdk.client.Robot` wraps the local API, typed (`Task`, its `Outcome`, `StopResult`, `StopStatus` are `TypedDict`s) and with the
standard library only:

```python
from roborun_sdk.client import Robot, RobotError, RobotUnavailable

robot = Robot()                                   # http://127.0.0.1:8120
task = robot.run('pick up the cup', on_update=lambda t: print(t['status'], t['message']))
print(task['status'], task['moves'], task['message'])
held = (task['outcome'] or {}).get('holding', {}).get('left')   # {'object': 'cup', ...}, None (empty) or 'unknown'
jpeg = robot.frame('top')                         # faces blurred
print(robot.state()['arms']['left']['joints_rad'])
robot.stop(); print(robot.stop_status()['state'])
```

Every call raises `RobotError` with `status`, `code`, `message` and `retryable` from the error above, and `RobotUnavailable`
(status 0, code `not_running`) when nothing answers: Roborun is not running, start it with `roborun up`. Both are
`roborun_sdk.errors.RoborunError`, the base of every error the SDK raises to your code (an arm driver that cannot load is an
`ArmLoadError`). `run()` stops the task if waiting is interrupted (Ctrl-C, `timeout`). The same `stop()`, `stop_status()`,
`status()`, `health()` and `frame('head')` work on a humanoid profile (`frame('head')` is not face-blurred yet).

Every method: `status()`, `health()`, `state()`, `frame(camera, blur=True)`, `start(prompt, mode, arm)`, `task(id)`,
`tasks(limit)`, `live(id)` (an iterator of task views), `wait(id, timeout, on_update)`, `run(prompt, mode, arm, timeout,
on_update)`, `stop_task(id)`, `recording(id)`, `body_move(prompt, duration_s)`, `body_move_status(id)`,
`body_run(prompt, duration_s, timeout, on_update)` (a humanoid move, waited for), `stop()`, `stop_status()`, and
`call(method, path, body)` for any other `/v1` route.

[examples/robot_client.py](../examples/robot_client.py) is a small command line on top of it:

```sh
roborun python examples/robot_client.py run "pick up the cup" --stop-after 12
roborun python examples/robot_client.py stop
roborun python examples/robot_client.py frame top top.jpg
```

## An LLM's tools

[examples/llm_tools.py](../examples/llm_tools.py) gives an LLM the robot as five tools: `robot_status`, `look` (a camera
frame, faces blurred, as an image block), `arm_state`, `run_task` (one physical task, waited for) and `stop`. `TOOLS` holds
their JSON Schema specs for your LLM's tool-calling API, and `call_tool(name, arguments)` runs a call and returns what to
send back. Errors come back as `{"error": {"code", "message", "retryable"}}` instead of raising, so the model can read them
and decide.

```python
from llm_tools import TOOLS, call_tool
# give TOOLS to your LLM; for each tool call it makes:
result = call_tool(name, arguments)               # then send `result` back as the tool result
```

```sh
roborun python examples/llm_tools.py tools                       # the specs
roborun python examples/llm_tools.py call look '{"camera": "top"}'
```

### When a task fails

A `failed` task whose cause is known carries `error_info`, for code to branch on (it is `null` otherwise):

```json
{"status": "failed", "message": "The Roborun API is busy or restarting", "error": "The Roborun API is busy or restarting",
 "error_info": {"code": "service_unavailable", "retryable": true, "retry_after_s": null, "next": "run the task again in a minute"}}
```

| `code` | `retryable` | What happened |
| --- | --- | --- |
| `network_offline` | yes | The Roborun API could not be reached from this computer |
| `service_unavailable` | yes | The API answered 5xx (busy, restarting) |
| `service_timeout` | yes | The API did not answer in time |
| `rate_limited` | yes | Too many requests for this key: wait `retry_after_s` |
| `key_revoked` | no | The key is unknown or revoked: `roborun login`, then `roborun up` |
| `key_read_only` | no | A read-only key: robots need a robot key |
| `signature_invalid` | no | A reply did not pass its signature check: it was not used and nothing moved |
| `signing_key_unknown` | no | A reply was signed with a key this robot does not know: `roborun up` fetches the current keys |
| `sdk_upgrade_required` | no | The API no longer supports this Roborun version: `roborun upgrade` |
| `insecure_url` | no | The configured server is plain http on another computer |
| `object_not_measured` | no | No camera measured the object: make sure the top camera sees it (an arm without wrist cameras needs the overhead camera: `roborun setup`) |
| `no_depth_camera` | no | Tasks need a depth camera (e.g. RealSense) as the overhead camera; a webcam only shows and records. Plug one in, then `roborun up` |
| `task_failed` | yes | Any other failure: make sure the robot is clear and powered, see the task's `recording_url`, then run it again |

The connection itself rides out short outages: calls that are safe to repeat (reads, a job submit with the same id, a cancel)
are retried with backoff, a motion being made keeps being polled for up to three minutes without an answer (the task says
`Reconnecting to the Roborun API…` meanwhile), and a job the robot gives up on is cancelled at the API once it can be reached.
Nothing moves on a reply that does not verify against the API's signing keys.

### Staying up to date

Every call to the API names this Roborun's release, and every reply names the newest one. `GET /v1/status` carries
`"update": {"available": true, "latest": "<build>", "required": false}`, and `roborun up` / `roborun status` say
`update available: <build>, run: roborun upgrade` (at most once a day). `roborun upgrade` installs the newest release the way
install.sh does and restarts the robot on it, unless a task is running (then run `roborun up` after the task). A release the
API no longer supports gets `sdk_upgrade_required` on new work; reading the signing keys, signing in and Stop keep working.

## Versions and upgrades

Every Roborun release has two names, printed by `roborun --version`:

```
$ roborun --version
roborun-sdk 0.1.20261119172831 (build 20261119-172831-3f9c1e02ab)
robot runtime 2026.11.19+c18915e.092756
python ~/.roborun/python/bin/python
pin it: roborun-sdk==0.1.20261119172831  (pip install --find-links https://roborun.app/dl/simple/ roborun-sdk==0.1.20261119172831)
```

- **The version** is a PEP 440 version, `0.<series>.<UTC YYYYMMDDHHMMSS>`: the series (now `1`) and the time the release was
  built. Later releases always have higher versions, so `==`, `>=` and `~=0.1.0` work as usual. `roborun-sdk` requires
  exactly the `roborun-station` of the same version; you only ever pin `roborun-sdk`.
- **The build id**, `<YYYYMMDD-HHMMSS>-<digest>`, is the same release by the same time. The installer, the API's
  `update available` / `sdk_upgrade_required` and the download folders use it.

Releases are not on PyPI; they are at `https://roborun.app/dl/`:

```sh
roborun --version                                          # the version installed here
curl -fsSL https://roborun.app/install.sh | ROBORUN_VERSION=0.1.20261119172831 sh     # Roborun itself, at that release
pip install --find-links https://roborun.app/dl/simple/ "roborun-sdk==0.1.20261119172831"   # into your own environment
pip install --find-links https://roborun.app/dl/20261119-172831-3f9c1e02ab/ "roborun-sdk==0.1.20261119172831"   # one build only
```

`https://roborun.app/dl/simple/` lists every release (a find-links page, and a PEP 503 index: `--extra-index-url` works too;
prefer `--find-links` with an exact `==` pin). `roborun new` writes the pin into the project's `pyproject.toml`, with
`[tool.uv] find-links` set, so `pip install --find-links https://roborun.app/dl/simple/ -e .` or `uv sync` gets the same
Roborun. The other dependencies come from PyPI as usual.

What stays compatible within a series (every `0.1.*` release):

| Stable | Not stable (changes without notice) |
| --- | --- |
| The local API's `/v1` routes in [The local API](#the-local-api), their fields and the error shape: fields and routes are added, never renamed or removed. `GET /v1/openapi.json` describes the running release. | `/api/...` and `/internal/...` on port 8120 (the page's; `POST /api/stop` and `GET /api/stop/status` keep working) |
| `roborun_sdk.client` (`Robot`, `RobotError`, `RobotUnavailable`, `Task`, `StopResult`, `StopStatus`) and `roborun_sdk.errors.RoborunError` | Every other `roborun_sdk` module, and the station's own routes and ports |
| Your extension points: `StreamedArm` and its `*_hardware` methods, `open_arm`, the rules `check(side, positions_rad, gripper, state)`, the camera source class, the `robot.yaml` keys documented here | The robot runtime's files (it is replaced on every update) and the generated configuration |
| The `roborun` commands and flags documented here | Text meant for people: messages, log lines, `--help` wording |

An incompatible change to any of the stable column starts a new series (`0.2.*`) and comes as a new path (`/v2`) beside the
old one: `/v1` keeps answering through the next series, so a program written for `0.1` keeps running for one more series
while you move it. A pin like `roborun-sdk~=0.1.0` (any `0.1.*`) is therefore safe to upgrade within; an exact `==` pin is
what makes a build reproducible.

The hosted API is the other side of an upgrade: every reply names the newest release and the oldest one it still supports
([Staying up to date](#staying-up-to-date)). A release below that minimum can still stop, sign in and read the signing keys,
but new work gets `sdk_upgrade_required` until you run `roborun upgrade`. To upgrade a pinned project: `roborun upgrade`,
`roborun --version`, change the pin to the version it prints, and run the project's tests.

## Humanoids: Booster K1 and Galaxea R1

Two humanoid profiles ship with the SDK: `booster_k1` (legged, a head that pitches and turns, no arms) and `galaxea_r1`
(wheeled, two arms, a head camera fixed to the torso). They run like an arm profile, from the same command:

```sh
roborun up --profile booster_k1 --sim room                                     # simulation (scenes: room, empty)
roborun up --profile galaxea_r1 --body-agent http://ROBOT:8302 --body-agent-key-file AGENT_KEY   # a real R1 (agent: see below)
roborun up --profile yam_dual                                                  # back to arms (two YAMs)
```

A real body is reached through the small agent running on the robot (its address and key are remembered per profile; the
agent keeps its own deadman and speed caps, and must be armed on the robot). That agent is not public yet: to try Roborun
on a real K1 or R1, email support@roborun.dev. A humanoid that is neither connected nor set up to simulate does not
start: it is never replaced by a simulation. Both profiles appear in `roborun profiles` and in
`GET /v1/robot-profiles`, and the page switches to them like any other profile.

For a humanoid profile the local API at `http://127.0.0.1:8120` is:

| Call | What it does |
| --- | --- |
| `POST /v1/body/move` | Move: `prompt` (your words, as given), optional `duration_s` (2, 3, 4, 6 or 8; default 4). `202` with the move |
| `GET /v1/body/moves/{id}`, `GET /v1/tasks` | One move; recent moves |
| `POST /v1/stop` (also `POST /api/stop`) | Stop now, at any stage: the base stops, the head holds, a move still being made is cancelled |
| `GET /v1/stop/status` | Whether the body is at rest: `stopped`, `stopping`, `running` |
| `GET /v1/body/state` | `base`: odometry (`x`, `y`, `yaw`) and velocity; `head`: `pitch` and `yaw` |
| `GET /v1/body/frame.jpg`, `GET /v1/cameras/head.jpg` | The head camera now. Not face-blurred yet: keep it on this computer |
| `GET /v1/body/model` | The body's 3-D display model (`roborun.body-model/1`): its body tree with joints and the standing pose, visual meshes or shapes, `base_height_m`, the head joints that `head.pitch` / `head.yaw` drive, the head camera, and the simulated room (`null` on a real body). Pose it from `/v1/body/state`: the root at (`x`, `y`, `yaw`), `base_height_m` up. The page's 3-D view draws this |
| `GET /v1/body`, `GET /v1/status` | The profile (robot descriptor, camera, limits, `arms_offered`: the arms this API drives, none yet); the running move and the body's state |
| `GET /v1/health`, `GET /v1/docs`, `GET /v1/openapi.json` | Is it up; this list |

Errors have the same shape as on an arm profile (`{"error": {"code", "message", "retryable"}}`), and a move refuses fields
it does not know. `roborun_sdk.client.Robot` works here too: `body_move()`, `body_move_status()`, `stop()`, `stop_status()`,
`frame('head')`, and `body_run()` (a move, waited for).

A move takes the head frame (not face-blurred yet, see [Privacy](#privacy-and-your-data)), sends it with the prompt and the
robot's description (base, arms, head, speed and head limits, camera intrinsics and pose) to the hosted API as a body job,
and gets back a signed timeline. The robot checks the signature against the API's keys (`GET /v1/keys`) and that the reply is for exactly this
request before anything moves, then plays it at 50 Hz within the profile's speed and acceleration limits. The move record
says what came back:

```jsonc
{"id": "mv-3f2a...", "status": "done",          // running | done | stopped | failed (with error: {code, message})
 "stage": "done",                               // capturing, uploading, queued, planning, moving, then the status
 "prompt": "walk to the red box", "duration_s": 4, "job": "body-...", "verified": true,
 "rows": 45,                                    // the timeline: 10 rows a second of base velocity (vx, vy, vyaw) and head targets
 "summary": {"distance_m": 1.3, "heading_change_rad": 0.2, "low_confidence_frac": 0.04, "breaks": 1, "executable": true},
 "deferred": ["wave"],                          // parts of the prompt that did not fit this move, for your next one
 "warnings": [], "played": {"played_s": 4.6, "stopped": false, "odometry_start": {...}, "odometry_end": {...}}}
```

A humanoid prompt may be split up by the service: one move does what fits its length, and the rest is listed in
`deferred` for your next move. A move may play somewhat longer than `duration_s` (the plan sets its own pace). The humanoid
`POST /v1/stop` answers `{"stopped", "state" (stopping), "confirmed", "move", "stopped_tasks", "message"}`; follow it with
`GET /v1/stop/status` until `stopped`. To build on your own body, subclass `roborun_sdk.body.BaseAdapter` (velocity
`command`, `stop`, `state`) and `HeadAdapter` (`command`, `state`, `frame`); `roborun_sdk.body.sim.BodySim` is a complete
example.

## Your own interface

app.roborun.app is Roborun's page and runs no code from your robot; [the viewer](#the-viewer-and-the-page) is this robot's
own read-only view. Build your own interface on the local API: a local web page
served by your own program, a command-line tool, or an LLM agent that calls `http://127.0.0.1:8120`.

## What not to modify

These are Roborun's, and are replaced on every update. Changing them breaks updates and can break the safety guarantees:

- **The installed packages** (`roborun_sdk`, `roborun_station` in Roborun's Python) and **the robot runtime** in
  `~/roborun/robot/runtime`. Extend through the points above instead.
- **The generated configuration** in `~/.config/roborun/`: `robot.json`, `robot/jobs.json`, `robot/rig.json`. Use
  `roborun up` flags, `robot.yaml` and `roborun profiles create`. `robot.yaml` is yours: Roborun keeps your edits.
- **The station and its ports** (127.0.0.1:8091, 8120, 8135): don't bind them or call the station directly. Programs
  use the local API; the page uses the connector. The station's routes are internal and change without notice.
- **The safety layer**: the shaping, guards and Stop in `StreamedArm` and the station. Tune it with the arm's settings in
  `robot.yaml` (`joint_speed_rad_s`, `temp_stop_c`, ...; the comments in `robot.yaml` list them) and add your own limits with rules.
- **Face blur**: frames leave the computer only through the camera service, which blurs them (a humanoid's head frame is
  the exception, not blurred yet). Don't upload camera frames from your own code around it.

## Appendix: the hosted API

You never need to call the hosted API yourself: the robot runtime calls it for every task and verifies what comes back.
[HOSTED_API.md](HOSTED_API.md) covers checking your key from a script, its errors and routes, and verifying a signed result.
