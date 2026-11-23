# Developing roborun-sdk

This guide is for people changing the SDK itself. To build on Roborun without changing it (your own arm, camera, rules,
profile or program), read [docs/BUILDING.md](docs/BUILDING.md) instead.

## What the SDK is

The SDK is everything that runs on the computer wired to the arms. `roborun up` starts these processes, each as a user
service (systemd where it is available, plain child processes elsewhere):

| Process | Address | What it does |
| --- | --- | --- |
| `roborun-station` (or `roborun-sim`) | 127.0.0.1:8091 | The arms: your drivers wrapped in the station (motion leases, Stop and hold, Home), or the MuJoCo simulation |
| `roborun-cameras` | Unix socket | The cameras, with face blur where frames leave the computer |
| `roborun-robot` | 127.0.0.1:8120 | The robot runtime: the local API, tasks, recordings; asks the hosted API (`https://api.roborun.app`) for motion |
| `roborun-local` | 127.0.0.1:8135 | The connector that lets https://app.roborun.app, open in a browser on this computer, reach this robot |

The robot runtime is a separate, versioned bundle that `roborun up` downloads into `~/roborun/robot/runtime`; it is not in
this package. Motion comes from the hosted API: the runtime sends the camera view and the arms' state and plays back what
comes back. Safety, Stop and the arm drivers are local and live here.

## Layout

```
src/roborun_sdk/
  cli.py              the `roborun` command: registers every subcommand
  robot.py            roborun up / down / status / setup / profiles: installs the runtime, writes its config, starts the services
  guided.py           the guided first-run setup
  scaffold.py         roborun new: the starter project for integrations
  client.py           roborun_sdk.client.Robot: the typed client for the local API (public; keeps its docstrings in the wheel)
  errors.py           RoborunError, the base of every error raised to programs; ArmLoadError
  devkit.py           roborun --version, roborun docs, roborun python
  embodiment.py       ArmAdapter (the driver interface), KinematicArm, run_station, `roborun station`
  arms/               the arm catalog and drivers
    __init__.py         catalog (data/arms/<id>.json), open_arm(): catalog arms and own drivers (file.py:Class)
    base.py             StreamedArm: 100 Hz jerk-limited stream, hold/Stop braking, rules, guards, parking, dry run
    safety.py           thermal, stall and jaw guards, and the names of their settings
    settings.py         robot.yaml: template, loading, `roborun settings`
    hardware.py         robot.yaml -> the station; `roborun arms`, `roborun arm-check`
    connect.py, canbus.py, identify.py, calibrate.py, direction_check.py, packages.py
    yam.py, damiao.py, robstride.py, feetech.py, dynamixel.py, motor_arm.py, mit_can.py, serial_servo.py
    vendor_*.py         arms driven through their maker's SDK (UR, Franka, xArm, Kinova, KUKA, Piper, ...)
  cameras.py          `roborun cameras`: RealSense, USB, network/file streams and your own camera classes; the camera broker
  camera_*.py         capture helper, exposure, face blur, camera role checks
  overhead.py         the overhead camera's pose as reported by the robot
  arm_profiles.py     arm profiles from the catalog models; any pair of arms
  profile_create.py   roborun profile create --urdf: a profile for your own arm
  mujoco_sim.py       roborun sim: the profile as a MuJoCo simulation behind the same station API
  body/               humanoid profiles (Booster K1, Galaxea R1): BaseAdapter / HeadAdapter, their simulation, the
                      body-job client, and the local robot API a humanoid profile runs on :8120
  local_proxy.py      the page connector on 127.0.0.1:8135
  datasync.py         task records and recordings to the account (off with --no-data-sync)
  data/               arm catalog, arm models (MuJoCo + meshes), profiles, bundled models
  third_party/i2rt/   the vendored YAM driver stack (see its VENDORED.md)
  rt/, link/, connectors/, dev.py
                      the signed-segment runtime (`roborun doctor | run | rt | clear-stop`). Not used by `roborun up`;
                      kept with its tests while the segment wire format is still supported
examples/             runnable examples for integrators (see examples/README.md)
docs/BUILDING.md      the integrator guide; docs/HOSTED_API.md: what a developer may call on the hosted API
                      (README, docs/ and examples/ ship in every wheel as roborun_sdk/_docs: setup.py copies them in)
tools/                arm_models/ (rebuilds data/arm_models from makers' models; needs the robot runtime)
tests/                pytest suites; fakes for every motor bus (fake_*.py) and for the station owners; tests/data/*.segment.json:
                      signed sim segments (DEV keys) for the rt tests
```

The station logic around a driver (`roborun_station`) is its own package, `../roborun-station`, installed as a
dependency.

## Set up

Python 3.10 or newer. From this folder, in a virtual environment:

```sh
python -m venv .venv && . .venv/bin/activate
pip install -e ../roborun-station -e '.[test]'
```

Or, without installing, put the source trees on the path: `PYTHONPATH=src:../roborun-station/src`.

## Run the tests

Run every suite without network, so nothing can reach a real robot or a real service on this computer. The
selection is the one CI runs: it leaves out the guided-setup suite.

```sh
unshare -rn sh -c 'ip link set lo up; PYTHONPATH=src:../roborun-station/src python -m pytest -q tests/ \
  --ignore=tests/test_guided_setup.py'
unshare -rn sh -c 'ip link set lo up; PYTHONPATH=src:../roborun-station/src python -m pytest -q tests/test_scaffold.py'
```

`tests/test_guided_setup.py` drives the real `roborun` command against simulated hardware on virtual CAN links: it runs
only as root inside the namespace (which `unshare -rn` gives you) with the `vcan` kernel module loaded, and it installs
optional packages from PyPI, so it needs the network. CI runs it as a separate, non-blocking step.

`unshare -rn` gives the tests their own loopback: servers they start bind there, and the arms' and robot's real ports
(8091, 8120, 8135) on this computer are out of reach. Tests use the fakes in `tests/` (motor buses, station owners), never
hardware. Tests that need MuJoCo or the station package skip themselves when those are missing.

## Run it from source

**On a computer where a robot already runs under your user, a separate home is not enough.** `roborun up`, `down`,
`setup` and `station` start and stop systemd user units with fixed names (`roborun-station`, `roborun-robot`,
`roborun-cameras`, `roborun-local`, `roborun-sim`), whatever `HOME` says. A source run that reaches your systemd user
manager stops that robot's units, and its station may park the arms first. Best: run from source on another computer
(or a VM). If you must use this one, hide systemd from the run as well: no session bus, a private runtime folder, and
`systemctl` / `systemd-run` stand-ins that print nothing and fail, so Roborun runs its parts as plain background processes
(logs in `$HOME/.config/roborun/logs/`):

```sh
TOKEN=~/roborun-token; export PYTHONPATH=$PWD/src:$PWD/../roborun-station/src
export HOME=$(mktemp -d) && export XDG_RUNTIME_DIR=$HOME/run && mkdir -m 700 $XDG_RUNTIME_DIR
unset DBUS_SESSION_BUS_ADDRESS
mkdir $HOME/bin && for t in systemctl systemd-run; do printf '#!/bin/sh\nexit 1\n' > $HOME/bin/$t; chmod +x $HOME/bin/$t; done
export PATH=$HOME/bin:$PATH
python -m roborun_sdk.cli up --robot my-sim --key-file $TOKEN --sim cup
python -m roborun_sdk.cli status
python -m roborun_sdk.cli down
```

`--sim cup` runs the profile as a MuJoCo simulation with a cup on the table; everything else (local API, page connector,
hosted motion) is the same as with real arms. It needs a Roborun key and the internet (the first start downloads the
robot runtime). Wrap the commands in `unshare -rn` plus a user-mode network (for example
`pasta --config-net -t none -u none -T none -U none -- ...`) to keep ports 8091, 8120 and 8135 private while the hosted API
stays reachable.

Without any service running:

```sh
python -m roborun_sdk.cli arms                       # the catalog
python -m roborun_sdk.cli new demo --dir /tmp && python /tmp/demo/check.py
python -m roborun_sdk.cli station --adapter roborun_sdk.embodiment:KinematicArm --port 0
```

## Versions and releases

The source keeps `version = "0.1.0"` in both pyproject.toml files; only release builds set a real version. A release
has a build id `<UTC YYYYMMDD-HHMMSS>-<digest of the sources>` (`roborun --version`) and the PEP 440 version
`0.1.<YYYYMMDDHHMMSS>` from the same time; `roborun-sdk` requires `roborun-station` of the same version. Build ids compare as
strings and versions as numbers in the same order. `ROBORUN_VERSION=<version>` pins the installer to one release.

The series (`0.1`) is the major.minor of the source pyproject.toml. Raise it, in both packages together, only for an
incompatible change to what docs/BUILDING.md lists as stable ("Versions and upgrades"); such a change comes as `/v2` beside
`/v1`.

## Adding an arm to the catalog

1. `src/roborun_sdk/data/arms/<id>.json`: label, driver (`module:Class`), options, home and limits, the packages it needs.
2. A driver: a table-driven bus arm (`motor_arm.py` + a table) or a `StreamedArm` subclass (`vendor_base.py` for maker SDKs).
3. Its model in `data/arm_models/<id>/` (`tools/arm_models/build.py` from the maker's URDF or MJCF), so it gets a profile.
   The model tools (`build.py --kin`, `sim_cup_check.py --runtime`) need the robot runtime, which `roborun up` downloads
   with a Roborun key (`--kin ~/roborun/robot/runtime`). Without one, open the pull request with a link to the maker's
   URDF or MJCF, its commit and its license, and a maintainer builds the model.
4. Tests against a fake bus (`tests/fake_*.py`), the shared contract (`tests/arm_contract.py`) and `tests/test_catalog_profiles.py`.
5. Mark it `beta` until it has run on real hardware.

## Rules for changes

- Nothing moves on connect, on a failed read, or on a Stop: keep the guarantees listed at the top of `arms/base.py`.
- Stop must stay prompt: never add a blocking call in front of it.
- Every robot command shaped by `StreamedArm` stays smooth (speed, acceleration and jerk limited); don't send raw steps.
- New settings: add the name to `arms/safety.py` (or the driver's `EXTRA_SETTINGS`) and to the template in `arms/settings.py`.
- User-facing text is plain and says what to do next.
