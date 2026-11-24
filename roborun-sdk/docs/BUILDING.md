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
| Run your own mobile base or humanoid | [Your own robot body](#your-own-robot-body) | `~/.config/roborun/bodies/my_rover.yaml` + `roborun up --body /path/my_rover.py:MyRover` |
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
| Remove | `roborun uninstall` | It stops Roborun (no arm is dropped) and removes `~/.roborun`, the `roborun` command and the PATH line the installer added. It asks before deleting your settings, key, recordings and datasets (`~/.config/roborun`, `~/roborun`, with `~/roborun/datasets`): `--purge` deletes them, `--keep-data` keeps them, `--yes` does not ask. In a second install (`ROBORUN_HOME` set, or another `$HOME`) it removes only that install ([Two installs on one computer](#two-installs-on-one-computer)) |

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
| Stop motion now | `roborun stop` (also the Stop button and `POST /v1/stop`): it reaches the arms' station directly as well, so it stops a [policy run](#run-your-own-policy) even while Roborun itself is down |
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
  without systemd, in `~/.config/roborun/logs/`. A second install has its own names; see below.
- `port 8120 ... in use by another program`: stop that program (`lsof -i :8120`), then `roborun up` again. Roborun never
  stops a program it did not start to free a port.

### Two installs on one computer

Each Roborun home is one install: its settings, key, logs, services and camera socket. The default home is
`~/.config/roborun`. Set `ROBORUN_HOME` to another folder for a second install on the same computer account (a test robot
next to the real one, or a sandbox). Every `roborun` command run with that variable acts on that install only:

- **Services.** The default home's services keep their names: `roborun-robot`, `roborun-local`, `roborun-station`,
  `roborun-cameras` and `roborun-sim`. Any other home adds a short code made from its folder, for example
  `roborun-robot-1a2b3c4d`. `roborun status` shows the home and its service names
  (`journalctl --user -u roborun-robot-1a2b3c4d`). `up`, `down`, `status` and `uninstall` only start, stop or check that
  home's services. A changed `$HOME` on the same account also counts as another home, because systemd belongs to the account,
  not to `$HOME`.
- **Files.** Pid files and logs are in `<home>/run` and `<home>/logs`, the station's control files in `<home>/robot`.
  The camera socket is
  `$XDG_RUNTIME_DIR/manifest/camera.sock` for the default home and `$XDG_RUNTIME_DIR/manifest/roborun-<code>/camera.sock`
  for any other. If that folder is too deep for its sockets (a Unix socket path may be 107 bytes on Linux, 103 on macOS),
  as under a deep `XDG_RUNTIME_DIR`, they are in `/tmp/rr-<uid>-<12 hex>/` instead, a folder only you can open. Every
  part of the install works out the same path. A home set with `ROBORUN_HOME` (to a folder other than `$HOME/.config/roborun`) also keeps its runtime,
  recordings and datasets in `<home>/roborun/` instead of `~/roborun/`. A sandbox that changes only `$HOME` keeps them in
  its own `$HOME/roborun/`.
- **Ports.** Ports are shared by the whole computer. When a port the second install needs is taken, `roborun up` stops,
  names the port and says what to change. It never stops the other install. Give the second install its own ports
  with `roborun up` flags. They are saved in `<home>/robot.json`, so the next `roborun up` uses them too:
  - `--backend http://127.0.0.1:8220` for the local API (8120 by default).
  - `--live-port 8235` for the page connection and viewer (8135 by default). Use `0` to turn them off. The
    app.roborun.app page only connects to 8135, so only one install at a time can use the page.
  - The arms' station (real or simulated) always uses 8091, so only one install at a time can run arms. A humanoid
    profile doesn't use 8091. `roborun sim` alone uses 8191 (`--port` for another).
  - A simulation runs from the install's own copy of the runtime, `<home>/sim/`, even when `--runtime` names
    another install's runtime.
- **`roborun uninstall`** in a second install (`ROBORUN_HOME` set, or only `$HOME` changed: the same rule as the service
  names) stops that install, removes its home (when asked or with `--purge`) and, if it runs from its own folder, that
  folder. It never removes the account's `~/.roborun`, `roborun` command, PATH line, `~/roborun`, `~/.config/roborun` or
  the after-logout setting of the default install, also when you run it with the usual `roborun` command (which runs from
  `~/.roborun`). Here `~` is the account's own folder, whatever `$HOME` says. A home that holds the default install (for
  example `ROBORUN_HOME=~`) is left alone. It only removes an install folder that is inside its own home: a sandbox with
  its own `$HOME` (and `ROBORUN_HOME` unset, or `$HOME/.config/roborun`) that installed Roborun there with the installer
  removes its own `$HOME/.roborun`, command and PATH line. With `ROBORUN_HOME` naming another folder, the `$HOME/.roborun`,
  command, PATH line, `~/roborun` and `~/.config/roborun` of that `$HOME` belong to `$HOME`'s own install and are left
  alone too. Everything left alone is listed as `left alone: ...`.
- **`roborun upgrade`** with `ROBORUN_HOME` naming another folder stops without changing anything: the installer always
  installs into `$HOME/.roborun`, which is the software the default install's (or the sandbox's own) robot runs from. Run
  `roborun upgrade` without `ROBORUN_HOME` (it upgrades both and restarts that robot), then `roborun up` for the second
  install. A sandbox that changes only `$HOME` upgrades its own `$HOME/.roborun`, unless that folder, its
  `.local/bin/roborun` or a shell startup file is a symlink into the account's install (then it stops, as below).
- **Folders that lead back to the default install.** Before `up`, `setup`, `down`, `upgrade`, `uninstall` or `login` of a
  second install writes or deletes anything, it follows every symlink in the paths it would touch: its settings
  (`robot.json`), its station's control files (`<home>/robot/`: `down` asks the station named there to let go of its arms
  and signals its pid), pid files and logs, its runtime and data folder (also one named with `--runtime` or in
  `robot.json`), for `upgrade` the installer's `$HOME/.roborun`, `$HOME/.local/bin/roborun` and shell startup
  files, and for `login` the key file. If any of them leads into the account's default install (`~/.roborun`, the `roborun`
  command, `~/.config/roborun`, `~/roborun` or the PATH line's startup file), or holds it, the command stops with
  `roborun <command> stopped: ... Nothing was changed`, before it stops, asks or signals anything. This covers a sandbox
  whose `.roborun`, `roborun`, `.bashrc`, `.config` or `.config/roborun` is a symlink to the account's, a second home whose
  `robot`, `run` or `token` is, and a `ROBORUN_HOME` inside the default install (for example `~/.config/roborun/test`,
  `~/.roborun/x` or `~/roborun`). Give the second install folders of its own. A `ROBORUN_HOME` that holds the default
  install (`~`, `/home`) is not refused by `uninstall`, which leaves that home alone (see above).
- **What counts as the default install.** Only the account's own `~/.config/roborun`, reached the way the account reaches
  it: from the account's own `$HOME` (also a `$HOME` that is a symlink to the account's folder, and also when the account's
  `.config` is a symlink, as with dotfiles), or with `ROBORUN_HOME` set to that exact path (`/home/<you>/.config/roborun`)
  from any `$HOME`. That is the default install itself: it uses the plain service names, and `up`, `down`, `upgrade` and
  `uninstall` act on the running robot as usual. A sandbox with its own `$HOME` whose `.config` or `.config/roborun` is a
  symlink to the account's (or a `ROBORUN_HOME` that is a symlink to it) is a second install that borrows the default
  install's files, so every command above stops there. An empty `ROBORUN_HOME=` counts as unset, and a relative
  `ROBORUN_HOME` (or one with a quoted `~`) is made absolute from the folder you run `roborun` in: the services get that
  absolute folder, never a path they would resolve against the account's home.
- The default home is decided by the account's own folder (the password database), not `$HOME`. On a computer where
  `$HOME` is not the account's folder, the install there counts as a second one and gets suffixed names. If such an
  install was already running before this version, its services still have the plain names after the upgrade: stop them
  once with `systemctl --user stop roborun-robot roborun-local roborun-station roborun-cameras roborun-sim`, then
  `roborun up`. Do this only on that computer. Never run it from a sandbox on a computer whose default install is
  running: those are the running robot's services, and stopping the station may park the arms.

```
ROBORUN_HOME=~/roborun-test roborun up --profile booster_k1 --sim room --robot test --key-file ~/k \
    --backend http://127.0.0.1:8220 --live-port 8235
ROBORUN_HOME=~/roborun-test roborun status
ROBORUN_HOME=~/roborun-test roborun down
```

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

A simulation never runs in your live robot. It runs from its own copy of the robot runtime in `<Roborun home>/sim/`
(made on the first simulation without the runtime's `log/`, and refreshed when the runtime changes). The copy has its
own `log/` and `config.json` (it takes the runtime's `config.json` again only when that one changes), so the simulated
calibration, motion sessions and the simulated profile never reach the runtime your real arms use. `roborun up --sim`
runs the robot's backend and the simulation from that copy. The simulation is then the robot, so it uses the robot's
station port (8091) and camera socket. `roborun up --sim` refuses, and changes nothing, while your live robot's station
or camera service runs or a camera service serves that socket. Run `roborun down` first. Before it starts the
simulation it removes the install's own camera socket if nothing serves it (a left-over of a stopped camera service; the
camera service does the same when it starts). Going back with `roborun up --no-sim` (or `--left ...`) runs the runtime
and profile your arms had.

A humanoid simulation (`roborun up --profile booster_k1 --sim room`) follows the same rule. It refuses, and changes
nothing, while this install's arms station or camera service runs, or its robot service runs a real robot. Run
`roborun down` first: it parks or holds the arms, which stopping them from here would not.

`roborun sim` alone (a developer command: the simulated station and cameras without the robot's backend) runs from the
same copy on a port and socket of its own: the station on `127.0.0.1:8191` and the cameras on `sim-camera.sock` beside
the robot's camera socket. It asks no backend for motions unless you give one with `--predict URL`. `--port` and
`--camera-socket` change them. It refuses a port or socket that something already serves, and never removes a socket
that a Roborun simulation did not make. It never takes the robot's port 8091 or a robot's `camera.sock` (this install's
or another's), even while they are free: the live robot may only be between restarts. To simulate as the robot, run
`roborun down`, then `roborun up --sim <scene>`. `--runtime FOLDER` simulates another runtime (a checkout, another
install's): it runs from its own copy of that folder in `<Roborun home>/sim/runtimes/`, and the folder is only read.

## Privacy and your data

Camera images stay sharp on this computer: the robot sees them that way, and so do your recordings and the live view. Every
image or depth frame that leaves this computer has faces blurred first, as a best effort: frames sent for motion, and your
history. If the blur cannot run, nothing is sent, and the robot can still see and stop. `privacy: {face_blur: false}` in
`robot.yaml` sends frames for motion unblurred and keeps history images and depth on this computer (task records are still
uploaded). A policy run with a remote provider sends blurred frames to that provider, after you agree; see
[Run your own policy](#run-your-own-policy) and [Run a DreamLabs policy](#run-a-dreamlabs-policy).

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
pinned: roborun-sdk==0.1.20261124030404 (pyproject.toml)
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
The docstrings of these classes and methods ship in the wheel:
`roborun python -c "from roborun_sdk.arms.base import StreamedArm; help(StreamedArm)"` (likewise `ArmAdapter` in
`roborun_sdk.embodiment`, `CustomSource` in `roborun_sdk.cameras`, `BaseAdapter` and `HeadAdapter` in `roborun_sdk.body`).

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
    def open(self): return dict(serial='folder:frames')     # optional: serial, intrinsics, depth_scale_m, rectified
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

A Visio stereo device (General Intelligence Labs) has its camera class in the SDK: `pip install 'roborun-sdk[visio]'`,
then `source: roborun_sdk.visio_camera:VisioCamera` with `port: /dev/ttyACM0` (or `tcp://HOST:50001` over its USB network
link, or `mcap: run.mcap` for a recording). It serves the rectified left eye with depth from the device's own
calibration, and never writes to the device; the module's docstring lists every setting.

## Describe your cameras

Every camera has a card: where it is fixed, what image it delivers and which task camera it answers as. Wrist cameras and
cameras of any other kind are described the same way. `robot.yaml` names cameras `top`, `left` and `right` as before, and
you can also give a camera a name of your own (lower-case letters, digits and `_`). An optional `card:` section adds the
facts:

```yaml
cameras:
  top: realsense:SERIAL               # top, left and right are their own task camera, as always
  left:
    source: usb:ID
    rotate: 180                       # mounted upside down
    card: {depth: false}              # a statement: the camera's frames decide, a mismatch is a warning
  door:                               # a name of your own: viewed and recorded, not a task camera
    source: rtsp://HOST/live
    card: {label: Door camera, sees: [table]}
  stereo:                             # or a camera of your own name that answers as one of them
    source: ~/my-robot/stereo.py:StereoHead
    card: {alias: right, mount: right.tool, rectified: true}
```

| Card key | What it says |
| --- | --- |
| `alias` | `top`, `left` or `right`: the task camera this one answers as (names of your own only; each alias once) |
| `mount` | `world` (fixed to the table), `left.tool` or `right.tool` (rides that gripper). Default: from the alias |
| `label`, `device` | text for people |
| `depth` | `true` or `false`: what you expect. The frames decide; in `roborun sim` this decides |
| `rectified` | `true` when the source already serves undistorted (pinhole) frames |
| `pose` | your measured `T_parent_camera` (4x4). Reported only: the robot plans from its own calibration |
| `sees` | up to 8 short names, such as `table` or `left.gripper` |

A wrist camera plans only its own arm: `left.tool` can only be the `left` camera. A camera on the table can only be
`top`. Any other key in `card:` is refused, as are names like `overhead`, and every mistake is named in plain words
before anything starts. The `card` key never reaches your camera class. Every other key of the section still goes to
its `__init__`, exactly as before.

```sh
roborun cameras describe          # one line per camera; reads robot.yaml, the profile and the camera service
roborun cameras describe --json   # the cards: {"schema": "roborun.cameras/1", "profile", "table_frame", "cameras": [...]}
```

`describe` opens no camera, starts nothing and writes nothing, and it exits 1 on an invalid entry. A card follows
OpenCV's camera axes (x right, y down, z forward). Its pose is `T_parent_camera` in metres: the parent is the table frame
the planner uses for `world`, or that arm's grasp site for `<arm>.tool`. Its intrinsics are in pixels of the image as
delivered, after `rotate`. A pose, a pinhole or a calibration this computer cannot know is `null`. A pose taken from the
profile is marked `preview`, because the robot decides the pose it plans from. Calibration and planning (`calibration`,
`planning`) come from the robot, never from your file. Defaults are strict:

- A YAM wrist camera uses the YAM's design mount until it is calibrated. Every other arm's wrist camera (OpenArm, Metal,
  your own) has no pose until its calibration exists, so motion is not planned from it.
- A design or nominal pose is for the camera unrotated. A camera served with `rotate` gets no pose until it is calibrated
  as it is delivered.
- A camera that gives no depth says so (`depth.available: false`). Its frames carry zeros and no depth scale.

What leaves the robot for a wrist-camera task is camera facts only: its name, the pinhole intrinsics of the frame, its
pose at the exposure (`T_table_camera`, worked out on the robot), its mount on the gripper (`T_tool_camera`) and the
table height. Joints, serial numbers and paths never leave. Only a rectified frame goes: a frame whose intrinsics still
carry lens distortion is refused, even when its K equals the camera's pinhole.

In `roborun sim` (and `roborun up --sim`) the simulation draws the profile's cameras and a wrist camera on every arm,
with the size and `depth` of your `robot.yaml` cameras of the same alias that have a `card:` section. A camera entry
without a card leaves the simulation as it was, so a size written for your real camera does not change it. A
`depth: false` camera serves no depth, exactly like a real webcam. While the
simulation runs, `describe` shows the pose and intrinsics each camera is drawn with. Humanoids describe their head camera
from the body profile (`roborun cameras describe --profile booster_k1`).

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

## Run your own policy

New in this release; it may change within 0.1 (the names below are not on the stable surface yet).

A policy run is a Roborun task whose motion comes from a policy (your own model, or a provider's) instead of Roborun's
motion service. Roborun stays in charge of the arms: the station plays the policy's output through every limit, holds the
arms when the policy is late, and Stop ends the run at once. The run gets everything any task gets: a task record, a
recording and a dataset episode when this robot records its tasks, the viewer and History.

```sh
roborun policy providers                                          # the installed providers
roborun policy check --provider my_policy.py:Sway --arms left     # offline: does it fit this robot? nothing sent, nothing moves
roborun policy run --provider my_policy.py:Sway --arms left --seconds 20 --prompt "sway the wrist"
roborun policy run --provider my_policy.py:Sway --shadow --arms left    # asks the policy, moves nothing, reports what it would do
roborun policy runs                                               # past runs
```

```python
from roborun_sdk import policy

result = policy.run('my_policy.py:Sway', prompt='sway the wrist', arms='left', seconds=20)
result.task_id, result.status, result.stop_reason, result.summary
run = policy.start(MyPolicy(), prompt='wave', arms='left', seconds=60)   # in the background: run.stop(), run.wait(), run.events()
policy.check('my_policy.py:Sway', arms='left')                          # offline
```

Ctrl-C, `roborun stop`, the page's Stop and the viewer's Stop all end a run; the arms settle where they are and the record
says `stopped`. The task's own Stop works too: the page's Stop on the task and `Robot().stop_task(id)` stop that run at the
arms' station by its id, the same hold as an ordinary task's Stop (also while the model warms up, and while Roborun's backend
is down), and `Robot().stop_status()` and `wait(id)` count a running policy run as the running task (so do `Robot().status()`, `roborun status` and the viewer's "Now" line). The robot-wide Stop from `Robot().stop()`, the page and the viewer stops a policy run at the arms' station also when Roborun's backend is down, has not answered within 2 s, or answers with anything but a confirmed Stop (an error, a broken or non-JSON reply). The robot's own
`POST /v1/tasks/{id}/stop` at `127.0.0.1:8120` stops a policy run too, with the same hold, on the robot runtime released with
this SDK; an older runtime answers `404` for a policy run, so there use the Python client, `POST /v1/stop` or `roborun stop`. Exit codes: 0 done or stopped, 1 failed, 2 refused (nothing sent, nothing moved), 3 the model was ready too
late to start moving on its own.

Policy runs in simulation use `roborun up --sim <scene>` (after `roborun down`): `roborun policy run` then drives the
simulated arms on the robot's own station and reads the simulated cameras on the robot's camera socket, and the run's
records stay in the simulation's copy of the runtime (see [What is recorded, and where](#what-is-recorded-and-where)).
A bare `roborun sim` (station on 8191, cameras on `sim-camera.sock`) is not a target for a policy run: `roborun policy
run` has no station or camera socket option, so it always uses this robot's own station and cameras, and with the live
robot up those are the real arms.

### A provider

```python
from roborun_sdk.policy import Chunk, Provider, plain_needs

class Sway(Provider):
    def needs(self, spec):                        # offline: what it drives and wants to see; raise PolicyRefused if it does not fit
        self.arms = {a.side: a for a in spec.arms}
        return plain_needs(tuple(self.arms), frame_hz=2.0, version='sway 1')

    def start(self, spec):                        # optional: connect (after consent, for a remote provider). Nothing moves yet
        pass

    def act(self, obs):                           # rows of absolute targets from obs.t on; may take seconds (its own thread)
        rows = {}
        for side, arm in self.arms.items():
            q = list(obs.arms[side].commanded_rad)
            rows[side] = [q[:-1] + [q[-1] + 0.1 * i / 48] + ([1.0] if arm.gripper else []) for i in range(48)]
        return Chunk(rows, hz=30)
```

- `needs(spec)` gets the arms (`spec.arms`: side, catalog model, joint count, limits, whether it has a gripper), the camera
  roles that deliver frames (`top`, `left`, `right`), the prompt, the model name and your `--option` values. It returns a
  `Needs`: the sides it drives, the cameras it wants (`{its role: Roborun role}`), how often it wants observations, whether it is
  remote (frames and state leave this computer) and, for a remote provider, the plain words of what it sends. It must not
  touch the network: `roborun policy check` calls only this.
- `observe(obs)` (optional) is called at `frame_hz` so a provider can keep a history; `act(obs)` returns a `Chunk`. The
  observation holds per arm the measured joints, the target the station last played (`commanded_rad`) and the grippers, all
  from one pose read at `obs.t`, plus the newest camera frames as JPEGs.
- `act` raises `ProviderError` when it has nothing this time (`retryable`, `retry_after_s`, `warming` while a model warms up,
  `session_restarted` to end the run). Any other exception fails the run.
- `end_episode()` and `close()` (optional) run after the run, off the Stop path, at most 3 s.
- Load a provider by file (`my_policy.py:Sway`, relative paths also under the Roborun home), by module
  (`mypackage.policies:Sway`), or by name through an entry point in the group `roborun.policies`. `--option k=v` (Python:
  `options={...}`) goes to its constructor; an option it does not take is refused before anything is sent or moves.
- Built in: `hold` (keeps the arms where they are; a self-test) and `replay` (plays the `action` rows of a recorded episode:
  `--option path=episode.npz` or a dataset folder with `--option episode=N`; `--option relative=true` starts it where the arm
  is). [examples/my_policy.py](../examples/my_policy.py) is the provider above as a runnable file.

**Units and joint order.** Rows are absolute targets in the arm's own joint order (the order of `spec.arms[i].limits_rad`
and of the observation), in radians; the last value is the gripper as an open fraction, 0 closed to 1 open, for an arm with a
gripper. The station converts the fraction to the arm's own jaw units (a YAM's 0..1, a Metal's motor units). A provider that
works in other units converts them itself.

### What Roborun enforces after the policy

Before anything moves: the provider's `needs()`, then the match against this robot (sides, joint counts, grippers, cameras,
extras), then consent for a remote provider, then `start()`, then the warm-up. Only when the first chunk is there does the
run take a lease on the arms, and if that took more than 60 s after you started it, it asks again before moving (`policy.run`
raises `PolicyNotStarted` instead): the arms never start moving on their own after a long wait.

While it runs, the station (127.0.0.1:8091) checks every chunk and plays it on its own 30 Hz tick:

| Check | Limit | What happens |
| --- | --- | --- |
| Rows | finite, the right length, at most 512 per arm, 5-200 Hz, at most 10 s ahead | refused; the arm plays on what it had, or holds |
| Gripper | an open fraction within -0.05 .. 1.05 | refused |
| Age | the chunk's observation at most 3 s old; rows already due are skipped, never played late | refused when nothing is left |
| Seam | the first row still due within 0.5 rad of the target the arm is at | refused |
| Step per tick | 0.04 rad per joint, 0.081 on the jaw | clipped (counted) |
| Then, on every arm | the joint limits and speed (0.6 rad/s), jerk shaping at 100 Hz, your rules file | as for every task |
| Tool speed | 0.25 m/s by default (`policy: {max_tool_speed_m_s: ...}` in `robot.yaml`, at most the arm's own; `--max-tool-speed` lowers it) | slowed; needs the arm's model |
| An empty buffer | the policy is late | the arm holds its last target; 10 s dry ends the run |
| Five refusals in a row | | the run ends (`policy_error`) |
| The runner stops (crash, hang) | no heartbeat for 1 s | every leased arm holds; `roborun policy runs`, the next `policy run` and `roborun stop` mark the run `failed` (`runner_gone`) |
| Stop | the page, the viewer, `roborun stop`, `POST /v1/stop`; this task's Stop (the page, `Robot.stop_task`, `POST /v1/tasks/{id}/stop`) | the lease ends on the next tick and the arms hold |
| Stop while the model warms up | any Stop after the run started | the run ends `stopped` with nothing leased; the station refuses a lease asked for after a later Stop |
| SIGTERM or SIGHUP to `roborun policy run` | | as Ctrl-C: the lease ends, the arms hold, the record says `stopped` (`signal`) |

`--arms left` with a provider that drives both arms leases only the left arm: the right arm's rows are dropped before they
are sent, and the station refuses rows for an arm it was not given.

**Shadow runs.** `--shadow` takes no lease and moves nothing: it asks the policy for chunks against live observations for
`--seconds` (20 by default) and reports the largest seam (the first row due vs where the arm is), the largest step from the
measured pose, the fastest row and what the station would refuse, in a table and in `log/policy/<id>/shadow.json`. Run one
before the first run of a new model on real arms.

### What is recorded, and where

| What | Where | Uploaded with your history |
| --- | --- | --- |
| The task record (status, stop reason, Roborun's counts: chunks played, refusals, holds, clipped ticks) | `<runtime>/log/tasks/<id>.json`; `/v1/tasks`, the viewer and History list it | yes |
| The recording (cameras at about 10 Hz, joints at 30 Hz), when this robot records its tasks: by the same setting as every task's recording, the runtime's `config.json` `video_record` (`enabled` true; only the `cameras` it lists; the same retention as every task's recording: before the run, the oldest recordings, tasks' and policy runs' alike, are deleted until there is room for `run_cap_mb` under `total_cap_mb`; nothing when the disk has less than `min_free_mb` free, today's recordings reached `day_cap_mb`, or there is no room even then; during the run, no more frames past `run_cap_mb`, `day_cap_mb`, `min_free_mb` or a full disk, and the task record's `recording` says which cap (`capped`, `why`) and what was deleted (`cleanup`)). With the recorder off, a run leaves no recording and no episode, as every task | the runtime's recordings folder, beside every task's recording; `/v1/tasks/<id>/recording` | yes, faces blurred |
| A dataset episode (the same features and units as every other episode; `action` is the target the station played) | this robot's LeRobot dataset, only from a recording and only when the robot keeps one, by the same rule as every task's episode (on for an installed robot; `roborun up --no-dataset` or `--no-data-sync` turns it off; the runtime's `config.json` `lerobot_export.enabled` true or false overrides either way; no `robot.json`, no episode). `--no-dataset` skips it for one run and can never turn it on; a simulation (`roborun up --sim`, or cameras whose serials look simulated) never writes one | yes, faces blurred |
| The provider's timing (one line per request: round trip, sizes, age, outcome), every chunk as it came, a summary, the provider's own files | `<runtime>/log/policy/<id>/` | **no**: it stays on this computer |

Under `roborun up --sim`, `<runtime>` is the simulation's own copy of the runtime in `<Roborun home>/sim/`, as for every
simulated task: the simulating robot lists the run (`/v1/tasks`, the viewer, History), and the live runtime's `log/` and
recordings are left as they are.

Dataset grippers are in the station's units, as for every other episode of the same arm: 0..1 on YAM arms, motor units on a
Metal arm. A policy episode's extras say `gripper_units: "station"` and carry each arm's `jaw_range` (closed, open), so the
open fraction is `(gripper - closed) / (open - closed)`.

### Remote providers and consent

A provider whose `Needs.remote` is true gets frames and state off this computer, so Roborun asks before anything is sent,
including the model's warm-up: on a terminal it prints what will be sent and asks once per provider and model (the answer is
kept in `policy-consent.json` in the Roborun home, never in `robot.yaml`); `--yes` agrees for one run only; from Python,
`consent=True` agrees for that run. With no answer, the run is refused and nothing is sent. Turning face blur off asks again.
`roborun policy consent` lists what you agreed to, `--revoke PROVIDER[:MODEL]` forgets it. Frames sent to a remote provider
are face-blurred (unless `privacy: {face_blur: false}`); if the blur cannot run, nothing is sent and after 3 s the run ends.

### Run a DreamLabs policy

[DreamLabs](https://platform.dreamlabs.ai) makes Dream-1, a robot policy model, and publishes its Python client,
[`dreamlabs-client`](https://pypi.org/project/dreamlabs-client/) (Apache-2.0). With the `dreamlabs` provider, Roborun runs
a DreamLabs model as a policy on your arms. The model chooses the motion. Roborun keeps everything this section describes:
the limits and holds above, Stop, and the task record, recording, dataset episode and timing of each run. For your
DreamLabs account, keys, models and terms, see DreamLabs' own documentation.

```sh
pip install 'roborun-sdk[dreamlabs]'           # DreamLabs' client, plus numpy and OpenCV for the face blur
# set up your DreamLabs credentials as DreamLabs documents them: DREAM_API_KEY (and the DREAM_*_URL variables),
# or the file ~/.dreamlabs/credentials ($DREAM_CREDENTIALS names another)
roborun policy check --provider dreamlabs --model dreamlabs/dream-1-beta              # offline; says where the key comes from
roborun policy run --provider dreamlabs --model dreamlabs/dream-1-beta --shadow --prompt "fold the towel"
roborun policy run --provider dreamlabs --model dreamlabs/dream-1-beta --prompt "fold the towel" --seconds 60
```

From Python: `policy.run('dreamlabs', model='dreamlabs/dream-1-beta', prompt=..., consent=True)`;
[examples/dreamlabs_policy.py](../examples/dreamlabs_policy.py) has check, shadow and run.

**What it runs on.** The base model `dreamlabs/dream-1-beta` runs on two YAM arms with grippers and three cameras: `top`
(DreamLabs' `overhead`), `left` (`wrist_l`) and `right` (`wrist_r`). It reads both arms' joints, grippers and tool poses, so it
needs both arms even with `--arms left`; then only the left arm is leased and moves, and the right arm's rows are dropped. Roborun runs
it only on arm models whose joint convention (order, signs, zeros, limits, gripper 0 closed .. 1 open) was checked against
DreamLabs' own YAM configuration (`policy/data/dreamlabs_conventions.json`). Today that is the YAM. Any other robot is refused
by `check` and `run` before anything is sent. Run `--shadow` once before the first run of a model on real arms.

The tool pose of each arm (`state.<side>_eef_xyz`, the tool point in metres in that arm's base frame, and `_eef_rot6`, the
first two columns of its rotation) is computed by Roborun from the joint angles, in the tool frame DreamLabs' own IK
uses for the YAM (0.1347 m along link_6's z axis; `policy/dreamlabs_yam.py`, with DreamLabs' kinematics under Apache-2.0).

**What goes to DreamLabs, and when.** Only while a DreamLabs run or shadow run is active, after you agree, including while
the model warms up and before any motion. Roborun sends it to DreamLabs' inference service (`DREAM_GATEWAY_URL`, by default
`api.dreamlabs.ai`):
- the prompt, and the joint angles, grippers and tool poses (where each gripper is and how it is turned, computed from the
  joint angles) of both arms;
- up to 9 recent frames per camera (top, left, right) at 320x176, face-blurred (unblurred only if you set
  `privacy: {face_blur: false}`; the consent is then asked again and says so). This is about 1.3 MB per request at about one
  request per second, roughly 80 MB per minute;
- your DreamLabs key, as DreamLabs' client sends it, and an end-of-session call when the run ends. For a fine-tuned
  `dream-1-ft:...` model, DreamLabs' client also looks up what the model was trained on when the run starts.

Nothing else goes to DreamLabs. The run's Roborun task record, recording and dataset episode are not sent to DreamLabs;
like any task's, they go to your Roborun history (see [What is recorded, and where](#what-is-recorded-and-where);
`roborun up --no-data-sync` keeps them on this computer). The request timing in `log/policy/` never leaves this computer.
`roborun policy check` sends nothing. A refused or missing consent sends nothing.

**Consent.** On a terminal, `roborun policy run` shows that list and asks once per model; `--yes` agrees for one run only,
and `consent=True` does the same in Python. `roborun policy consent` lists what you agreed to;
`roborun policy consent --revoke dreamlabs` forgets it.

**DreamLabs' client, as Roborun sets it up.** Roborun creates `dream.DreamClient()` with no arguments, so the client finds
your credentials in DreamLabs' own order. Roborun never reads, stores or prints the key; `check` names only where the key
will come from. With no key at all (no `DREAM_API_KEY` and no credentials file), `check` and `run` refuse and nothing is
sent. The embodiment is passed to the client, so it does not look the model up before the run. DreamLabs' client
also has a recorder that streams each observation to DreamLabs. Roborun turns it off unless you pass
`--option record=true` (the consent text then says so). The setting it uses, `_record`, is internal to the client, and
Roborun will not start a run if a client version refuses it. The client's log goes to `log/policy/<id>/provider/` unless you
set `DREAM_LOG` or `DREAM_LOG_DIR`. When the client asks a person to check what the model sees (`python -m dream.check`),
Roborun shows that notice once per run.

Options (`--option k=v`): `timeout_ms` (10000), `obs_window` (9), `obs_fps` (5), `action_hz` (30), `execution_horizon`
(30), `sync_above_ms` (3000), `frame_size` (320x176), `record` (false), `embodiment` (a fine-tune's `embodiment.json`, below).

**Pacing.** DreamLabs' client decides when to ask for the next chunk. While the median of the last five round trips stays
under `sync_above_ms` it asks early, so the next chunk lands as `execution_horizon` rows of the current one have run, and
the arms move without a break. Above it, the client plays each chunk to the end and only then asks again, so the arms
hold for a round trip after every chunk. The client's own default is 700 ms; a warm dream-1-beta round trip over the
internet was measured at about 0.6 s (median) and 1.0 s (p90), so Roborun passes 3000 ms. Set it lower if your link is
slow and you would rather the arms stop and go than run on an older plan.

**How DreamLabs' answers are handled.** The model's rows (30 per second, 48 per answer, already smoothed by DreamLabs'
client) are played through every Roborun limit above. While DreamLabs warms a model up (`pool_warming`), nothing is leased and
the run waits up to `--wait-ready` (2 minutes by default, at most 30; a cold start can take longer), and the 60 s
start-motion rule applies. A rate limit or a busy service is retried after the wait DreamLabs gives; the arms hold
meanwhile. A model failure is retried three times in a row, then the run fails. A refused key or request fails the run with
DreamLabs' reason. A session DreamLabs restarted ends the run as `stopped` (`provider_session_restarted`); run it again.
`log/policy/<id>/requests.jsonl` keeps each request's round trip, the model's own time and the sizes, on this computer only.

**Fine-tune on your own episodes (optional).** The episodes Roborun records can be exported in the dataset format DreamLabs
accepts, uploaded to your own DreamLabs account and used to fine-tune a model that you then run through Roborun. Each step
runs only when you ask for it:

```sh
pip install 'roborun-sdk[dreamlabs-data]'
roborun dreamlabs export --episodes last:20             # writes a folder; sends nothing
roborun dreamlabs upload ~/.config/roborun/dreamlabs/exports/<time> --name towels-v1   # shows what and where, then asks
roborun dreamlabs finetune --dataset ds_... --suffix towels                             # asks, then queues the job
roborun dreamlabs status ftj_...                                                         # once ready: the model's id
roborun policy run --provider dreamlabs --model dream-1-ft:... --prompt "fold the towel"
```

- `export` reads this robot's dataset and writes `dataset/<session>/` (LeRobot v2.1 with `observation.state` and `action`
  only: joints in radians, each gripper as an open fraction), `embodiment.json` (the parts and camera roles DreamLabs reads,
  and the arms they came from) and `validation.json`. It checks the result with DreamLabs' own offline check
  (`dream.datasets` `validate_dataset`) and exits non-zero when that fails. Every video is face-blurred first. An episode whose
  video cannot be blurred is left out and listed, and so is an episode shorter than 1.6 s.
- `upload` sends the export's `dataset/` folder to the DreamLabs data service your credentials name, after showing its size,
  episodes and cameras. `--yes` skips the question. Uploads and jobs are listed in `dreamlabs-uploads.jsonl` in the Roborun
  home (on this computer only).
- `status` shows a job or a dataset. Once a fine-tune is ready, Roborun remembers which arms it was trained on, so
  `--model dream-1-ft:...` runs on those arms. On another computer, pass `--option embodiment=<export>/embodiment.json`.

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
| `POST /v1/tasks` | Start a task: `prompt` (1-1500 characters), optional `mode` (`run` moves the arms, `preview` does not), `arm` (`left`, `right`, `both`; default: chosen from the prompt and the scene), `camera` (the camera the motion is generated from: `top`, the default, or the `left` or `right` wrist camera, which plans its own arm). `model` may be left out: there is one model, Manifest (`manifest-1-beta`) |
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
`call(method, path, body)` for any other `/v1` route. `clip(id)` is left only to say it was removed: it raises `RobotError`
code `removed` (`roborun docs changelog`). `help(Robot)` shows each method's docstring. `start()` and `run()` also take
`camera=`, as `POST /v1/tasks` does.

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
roborun-sdk 0.1.20261124030404 (build 20261124-030404-51f94cfe2b)
robot runtime 2026.11.23+27791c3.195653
python ~/.roborun/python/bin/python
pin it: roborun-sdk==0.1.20261124030404  (pip install --find-links https://roborun.app/dl/simple/ roborun-sdk==0.1.20261124030404)
```

- **The version** is a PEP 440 version, `0.<series>.<UTC YYYYMMDDHHMMSS>`: the series (now `1`) and the time the release was
  built. Later releases always have higher versions, so `==`, `>=` and `~=0.1.0` work as usual. `roborun-sdk` requires
  exactly the `roborun-station` of the same version; you only ever pin `roborun-sdk`.
- **The build id**, `<YYYYMMDD-HHMMSS>-<digest>`, is the same release by the same time. The installer, the API's
  `update available` / `sdk_upgrade_required` and the download folders use it.

Releases are not on PyPI; they are at `https://roborun.app/dl/`:

```sh
roborun --version                                          # the version installed here
curl -fsSL https://roborun.app/install.sh | ROBORUN_VERSION=0.1.20261124030404 sh     # Roborun itself, at that release
pip install --find-links https://roborun.app/dl/simple/ "roborun-sdk==0.1.20261124030404"   # into your own environment
pip install --find-links https://roborun.app/dl/20261124-030404-51f94cfe2b/ "roborun-sdk==0.1.20261124030404"   # one build only
```

### Install only what you need

`pip install roborun-sdk` is the light SDK: the client (`roborun_sdk.client`), your own drivers and rules (`StreamedArm`,
`ArmAdapter`), the camera and body interfaces and the `roborun` command, with three small dependencies (PyYAML, packaging,
roborun-station). Importing it never loads numpy, MuJoCo or OpenCV, and it imports on Windows too. Heavy and hardware packages
are extras; a missing one fails with the line that adds it, for example `mujoco is not installed: pip install
'roborun-sdk[urdf]'`.

| Extra | Adds | For |
| --- | --- | --- |
| `robot` | numpy and MuJoCo (exact pins), OpenCV, Pillow, msgpack, websockets, zstandard, cryptography, the USB-CAN and serial packages, the YAM driver's | everything a robot runs: `roborun up`, the simulator, cameras, the YAM and USB-CAN/serial arms. The installer (and `roborun upgrade`) install `roborun-sdk[robot]`; `roborun up` adds pyrealsense2 and an arm maker's SDK when they are first needed |
| `sim` | MuJoCo, numpy, Pillow | simulated arms and bodies |
| `urdf` | MuJoCo, numpy | `roborun profiles create --urdf` |
| `cameras` | numpy, OpenCV, pyrealsense2 | RealSense and USB cameras |
| `link` | msgpack, numpy, websockets, zstandard, Pillow | the session link (`roborun_sdk.link.codec`, `capture`) |
| `crypto` | cryptography | fast Ed25519 checks (a pure-Python fallback runs without it) |
| `dataset` | pyarrow, PyAV, numpy | the LeRobot dataset (`roborun up` installs it once) |
| `yam`, `damiao`, `robstride`, `feetech`, `dynamixel`, `piper`, `trossen`, `xarm`, `ur`, `franka`, `abb`, `arx`, `rosbridge` | that arm's packages | `roborun up` installs the one an arm needs the first time it is used |

```sh
pip install --find-links https://roborun.app/dl/simple/ "roborun-sdk[robot]==0.1.20261124030404"   # the whole robot
```

Releases built before the extras existed (0.1.20261121012822 and older) install everything with plain `roborun-sdk`.

`https://roborun.app/dl/simple/` lists every release (a find-links page, and a PEP 503 index: `--extra-index-url` works too;
prefer `--find-links` with an exact `==` pin). `roborun new` writes the pin into the project's `pyproject.toml`, with
`[tool.uv] find-links` set, so `pip install --find-links https://roborun.app/dl/simple/ -e .` or `uv sync` gets the same
Roborun. The other dependencies come from PyPI as usual.

What stays compatible within a series (every `0.1.*` release):

| Stable | Not stable (changes without notice) |
| --- | --- |
| The local API's `/v1` routes in [The local API](#the-local-api), their fields and the error shape: fields and routes are added, never renamed or removed. `GET /v1/openapi.json` describes the running release. | `/api/...` and `/internal/...` on port 8120 (the page's; `POST /api/stop` and `GET /api/stop/status` keep working) |
| `roborun_sdk.client` (`Robot`, `RobotError`, `RobotUnavailable`, `Task`, `StopResult`, `StopStatus`) and `roborun_sdk.errors.RoborunError`. `Robot.clip()` was removed in 0.1.20261119224249 (a break in this series); it now raises `RobotError` code `removed`, and `Robot.recording(id)` is what the cameras saw | Every other `roborun_sdk` module, and the station's own routes and ports |
| Your extension points: `StreamedArm` and its `*_hardware` methods, `open_arm`, the rules `check(side, positions_rad, gripper, state)`, the camera source class, the `robot.yaml` keys documented here. Camera cards: the `roborun.camera/1` fields and `roborun_sdk.camera_card` (`check_config`, `derive`, `body_card`, `describe`, `validate`, `outbound`, `normalise_intrinsics`, `nominal_pinhole`) | The robot runtime's files (it is replaced on every update) and the generated configuration |
| The `roborun` commands and flags documented here | Text meant for people: messages, log lines, `--help` wording |

Every release is in the changelog, `CHANGELOG.md` beside the README (`roborun docs changelog` prints it), and anything
in the stable column that is removed or changed is listed there under **Removed** or **Changed**. The SDK's own test
suite runs `tools/stable_surface.py`, which compares the stable Python names and signatures with the last release and
fails on a removal or change that the changelog does not name, exactly and in backticks, under **Removed** or
**Changed**. Your driver and your program run on the SDK installed in
Roborun's own Python, not on the pin in your project: `roborun upgrade` (or a release the hosted API no longer supports)
changes it, so read the changelog before you upgrade.

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
`GET /v1/stop/status` until `stopped`. To run your own body (a rover, another humanoid), see
[Your own robot body](#your-own-robot-body).

## Your own robot body

A mobile base or a humanoid of your own runs from two files of yours, without changing Roborun: a **profile** that
describes the body, and a **class** that drives it. The same local API, player, obstacle guard, Stop, recordings and page
then work for it as for the packaged bodies. `examples/rover.py` is a complete one: a diff-drive rover with one fixed
camera (`examples/my_rover.yaml`, `examples/my_rover.xml`).

```sh
roborun docs --examples ~/my-rover && cd ~/my-rover
roborun python rover.py                     # offline: drives the rover through Roborun's player, prints what a move would send
roborun python rover.py --mujoco            # the same in the MuJoCo simulation of my_rover.xml, in the room
roborun python rover.py install             # copies my_rover.yaml and my_rover.xml into ~/.config/roborun/bodies
roborun up --profile my_rover --sim room                      # the rover simulated, its local API on 127.0.0.1:8120
roborun up --profile my_rover --body ~/my-rover/rover.py:Rover   # your class, on your robot
roborun up                                                     # afterwards: the same again
```

### The profile

Your profiles are files in `~/.config/roborun/bodies/` (`<ROBORUN_HOME>/bodies/`): `<id>.yaml`, `.yml` or `.json`, where
the file name is the profile's id (lowercase letters, digits, `_` and `-`; not a packaged body's id, and not a name that
means arms, such as `so101`, `yam_dual` or `so101+piper`). `roborun profiles` lists them, and names every file that cannot
be used with the reason. A profile is checked when it is read: a mistake stops `roborun up` before anything
starts, with the file and the field (`my_rover.yaml: camera.eye_height_m: between 0.1 and 3.0 (got 7)`). The motion service
plans every move from what the profile says, so give the real numbers of your robot.

```yaml
label: My rover
kind: wheeled              # wheeled (also: rover) or humanoid
base: wheeled              # wheeled or legged; a wheeled kind has a wheeled base, a humanoid says which
arms: 0                    # 0 to 2 (told to the planner; this API drives no arms yet)
head_dof: 0                # 0: a fixed camera, 1: it pitches, 2: it pitches and turns
limits:
  vx_m_s: [-0.2, 0.5]      # forward speed range, m/s; must include 0
  vy_m_s: [0, 0]           # sideways (default [0, 0]: a base that cannot strafe)
  vyaw_rad_s: [-1.0, 1.0]  # turning rate range, rad/s
  # head_pitch_rad: [-0.3, 0.8]   required with head_dof 1 or 2;  head_yaw_rad: [-1, 1]   required with head_dof 2
  # head_rate_rad_s: 1.5, deadband_m_s: 0, walk_min_pitch_rad: 0.3 (a walking head looks at least this far down)
camera:                    # the one camera every move is planned from
  size_px: [640, 480]
  hfov_deg: 90             # horizontal field of view: the intrinsics are made from it when your frame gives none
  eye_height_m: 0.35       # the camera's height above the floor (0.1 to 3 m)
  pitch_rad: 0.2           # its tilt, + looks down
  lens: a 90 degree USB webcam     # optional: words for the planner
footprint_m: 0.25          # the base's radius, for the obstacle guard (default 0.25)
motion:                    # optional; these are the defaults
  accel_m_s2: [0.5, 0.5]   # vx, vy
  accel_yaw_rad_s2: 1.0
  stop_decel_m_s2: 1.5
  stop_decel_yaw_rad_s2: 3.0
  deadman_s: 0.3           # a velocity target is fresh this long: the player resends it at command_hz
  command_hz: 50
sim:                       # optional: for --sim room|empty and the page's 3-D view
  model: my_rover.xml      # an MJCF file, relative to the profile: a base body with a <freejoint/>, and a camera
  base_body: base_link     # default: base
  head_camera: head        # the camera in the model (default: head)
  # head_joints: {pitch: ..., yaw: ...} for a head that moves; stance: {joint: angle} for a standing pose
# guard: {...}             # optional: the obstacle guard's settings, made from footprint_m and eye_height_m when left out
```

What a move sends the motion service about your body (`robot`: kind, base, arms, head_dof, limits; `camera`: intrinsics,
eye height, tilt, lens) comes from this profile and your camera's frame, never from a packaged body. A packaged profile
(`data/bodies/booster_k1.json`, with its `robot:` section) is valid here too, as a starting point; give your copy its own
name: a packaged id always means the packaged body.

### The class

Name your class like an arm driver: a file and a class (`/path/my_rover.py:MyRover`; in `robot.yaml` a relative path is
from robot.yaml's folder), or a module of a package installed in Roborun's Python (`acme_robots.rover:Rover`). Give it
with `--body` (with `--profile`), or in `robot.yaml`:

```yaml
body: my_rover.py:MyRover            # its profile: the class's own `profile = 'my_rover'`, else robot.yaml's profile:
# or, with settings for its constructor:
body:
  adapter: my_rover.py:MyRover
  profile: my_rover                  # this one wins over the class's attribute
  port: /dev/ttyACM0                 # every other key is a keyword argument of MyRover(...)
```

`body:` in robot.yaml is the class of that one profile. A bare `roborun up` (or `--profile my_rover`) runs it; `--profile`
of anything else (another body such as `booster_k1`, or arms such as `yam_dual`, also from the page's switch),
`--body-agent`, `--rig`, `--left`/`--right`, `--sim` with an arm scene (`--sim cup`) or `--body` leave it out, and it is
never remembered for another profile. Arms named that way also leave out robot.yaml's `profile:` when it names a body, so
`roborun up --rig rig.json` or `--sim cup` runs arms (robot.yaml's own `sim:` does not count: it says how the robot runs). With
no profile in the section, in the class or in robot.yaml, `roborun up` asks you to name one. `--body` refuses a class
whose `profile` attribute names another profile than `--profile` (`--body rover.py:Rover --profile booster_k1` would drive
a rover described to the planner as a K1); to use one class for several profiles, leave the attribute out, or name the
profile in robot.yaml's section.

Roborun makes one object of the class, in the robot service `roborun up` starts: `MyRover(profile=<the profile, a dict>,
**settings)` (the profile only when the constructor takes a `profile` argument; a setting it does not take is reported and
left out). The object is one of: an object with `.base` (a `BaseAdapter`) and `.head` (a `HeadAdapter`), and optionally
`close()`; one class that subclasses both; or a `BaseAdapter` with a `.head`. It is remembered per profile, like an agent,
so a bare `roborun up` (or the page's profile switch) runs it again; a class whose robot service does not start (its
constructor fails, say) is not remembered: `roborun up` stops that service, keeps the class the profile had before (if
any), and points to the log. `--profile` takes a profile id: `--profile rover.py:Rover` says the class goes on `--body`. `--sim` simulates the profile instead, from its
`sim.model`.

```python
from roborun_sdk.body import BaseAdapter, HeadAdapter

class Base(BaseAdapter):
    kind = 'wheeled'
    def command(self, vx, vy, vyaw): ...   # velocity target: m/s, m/s, rad/s; +x forward, +y left, +yaw counter-clockwise.
                                           # Fresh for deadman_s: the player resends it; stop the base when none comes
    def stop(self): ...                    # zero velocity now (brake or stand; never a power-off)
    def state(self):                       # odometry and measured velocity
        return dict(x=0., y=0., yaw=0., vx=0., vy=0., vyaw=0., commanded=dict(vx=0., vy=0., vyaw=0.), t=time.time())

class Camera(HeadAdapter):
    dof = 0                                # as the profile's head_dof
    def command(self, pitch, yaw): ...     # head targets, rad (+ pitch looks down, + yaw looks left); a fixed camera ignores them
    def state(self): return dict(pitch=0.2, yaw=0.)
    def frame(self): return dict(jpeg=...) # the camera now. Optional: width, height, intrinsics {width, height, fx, fy, ppx, ppy},
                                           # T_base_camera (4x4), eye_height_m, pitch, yaw: what is left out comes from the profile
    def depth(self): return None           # optional: {depth_m (HxW, metres), K {fx, fy, cx, cy}, T_base_camera, odom {x, y, yaw}}

class MyRover:
    profile = 'my_rover'
    def __init__(self, profile, port='/dev/ttyACM0'):
        self.base, self.head = Base(), Camera()
```

Without `depth()` a body has no obstacle guard: every move is slow (`blind_v_m_s`, 0.15 m/s) and goes at most
`blind_travel_max_m` (0.4 m), and `/v1/status` says so (`guard.obstacle_guard: false`). `GET /v1/body/model` needs the
profile's `sim.model`; without one it answers `503 model_unavailable` and the rest works. `/v1/status` gives `body_kind`
(`wheeled` or `humanoid`, from the profile) beside `kind: "humanoid"`, which stays the same for every body (it means "a
body profile's API", not the arm API, and existing clients test it): read `body_kind` for what the body is.

A body profile and arms are one or the other: arms on a mobile base are not supported yet. A robot-side agent speaking
the packaged K1 or R1 routes can also run with your profile (`agent: {protocol: booster|galaxea}` in it, and
`--body-agent`); the description sent is then your profile's too.

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
