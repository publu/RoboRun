# Roborun SDK

This is the latest iteration of RoboRun. Roborun turns a sentence and your camera view into motion on your robot arms.
The hosted service plans the motion; this SDK runs on the computer wired to the robot and keeps the arm drivers, cameras,
safety rules and Stop on your own machine. This repository holds the SDK and the station it runs your arms through, with
their docs, examples and tests. It is open source under the Apache License 2.0: use it, change it and build on it.

Earlier iterations of RoboRun (the `ros-agent` package, releases v0.1.0 to v0.9.0) are kept on the
[`legacy`](https://github.com/publu/RoboRun/tree/legacy) branch; their releases are tagged `legacy-v0.1.0` to `legacy-v0.9.0`
and their other branches are `legacy-feat-fleet-lab` and `legacy-openclaw-mcp`.

| Folder | What it is |
|---|---|
| [`roborun-sdk/`](roborun-sdk/) | The `roborun` command and the `roborun_sdk` Python package: `roborun up`, guided setup, arm drivers, cameras, MuJoCo simulation, the local API and its Python client |
| [`roborun-station/`](roborun-station/) | The station your arm drivers run inside: motion leases, Stop and hold, Home, plus simulated stations (pure Python) |

The robot runtime is not in this repository: `roborun up` downloads it from the service the first time it starts.

## Install

Everyone, the official installer (macOS and Linux; Windows: WSL2):

```sh
curl -fsSL https://roborun.app/install.sh | sh
```

Developers can install from source into their own Python 3.10+ environment:

```sh
# from a clone
git clone https://github.com/publu/RoboRun.git roborun-sdk && cd roborun-sdk
python -m venv .venv && . .venv/bin/activate
pip install ./roborun-station "./roborun-sdk[robot]"

# or straight from git (station first: the SDK depends on it)
pip install "git+https://github.com/publu/RoboRun.git#subdirectory=roborun-station" \
            "roborun-sdk[robot] @ git+https://github.com/publu/RoboRun.git#subdirectory=roborun-sdk"
```

`[robot]` is everything a robot runs (the simulator, cameras, the arm drivers), the same set the installer installs.
Without it, `pip install ./roborun-sdk` is the light SDK: the Python client, your own drivers and rules and the `roborun`
command, with no simulator or hardware packages. Or pick only the extras you need, for example
`pip install "./roborun-sdk[cameras,feetech]"`; the list is in
[`roborun-sdk/pyproject.toml`](roborun-sdk/pyproject.toml) and
[Install only what you need](roborun-sdk/docs/BUILDING.md#install-only-what-you-need). `roborun docs` prints the guides
from any of these installs.

Using the service needs a Roborun key. Roborun is invite-only: if you don't have a key, email support@roborun.dev.

## Quickstart in simulation

This needs a Roborun key and an internet connection: `roborun up` stops at `no key: run roborun login` without one, and its
first start downloads the robot runtime from the service.

```sh
roborun login                # prints a link: approve this computer on https://app.roborun.app
roborun up --sim cup         # two simulated arms and a cup on this computer; the first start downloads the robot runtime
curl -X POST http://127.0.0.1:8120/v1/tasks -H 'Content-Type: application/json' -d '{"prompt": "pick up the cup"}'
roborun stop                 # Stop is never refused
roborun down
```

Or open <https://app.roborun.app> on the same computer and type the task there. The step-by-step version is
[`roborun-sdk/README.md`](roborun-sdk/README.md).

### Without a key

These run on this computer alone, no key and no network (from the `roborun-sdk/examples/` folder):

```sh
python minimal_arm.py                  # the smallest arm driver, on a local station on a free port
python folder_camera.py FOLDER         # your own camera, from a folder of images
python keep_out_box.py                 # a rules file: the gripper stays out of a box
roborun new my-robot                   # a starter project: driver, rules and a task client
roborun docs                           # the guides; roborun docs --examples DIR copies the examples out
```

## Building on Roborun

The reference is [`roborun-sdk/docs/BUILDING.md`](roborun-sdk/docs/BUILDING.md):

- [Your own arm driver](roborun-sdk/docs/BUILDING.md#your-own-arm-driver)
- [Your own camera](roborun-sdk/docs/BUILDING.md#your-own-camera) and
  [Describe your cameras](roborun-sdk/docs/BUILDING.md#describe-your-cameras) (camera cards: mount, lens, depth;
  `roborun cameras describe`)
- [Rules](roborun-sdk/docs/BUILDING.md#rules) (keep-out zones and limits that run locally)
- [Arm profile from a URDF](roborun-sdk/docs/BUILDING.md#arm-profile-from-a-urdf)
- [The local API](roborun-sdk/docs/BUILDING.md#the-local-api)
- [The Python client](roborun-sdk/docs/BUILDING.md#the-python-client)
- [An LLM's tools](roborun-sdk/docs/BUILDING.md#an-llms-tools)
- [Humanoids: Booster K1 and Galaxea R1](roborun-sdk/docs/BUILDING.md#humanoids-booster-k1-and-galaxea-r1)
- [Your own robot body](roborun-sdk/docs/BUILDING.md#your-own-robot-body) (a mobile base or a humanoid of your own)
- [Profiles and simulation](roborun-sdk/docs/BUILDING.md#profiles-and-simulation) (`roborun sim` runs from its own copy
  of the runtime and never touches a live robot) and
  [Two installs on one computer](roborun-sdk/docs/BUILDING.md#two-installs-on-one-computer)

Also: [`roborun-sdk/docs/AGENTS.md`](roborun-sdk/docs/AGENTS.md) (a short guide for coding agents that install and drive Roborun),
[`roborun-sdk/docs/HOSTED_API.md`](roborun-sdk/docs/HOSTED_API.md) (the hosted API), and
[`roborun-sdk/DEVELOPING.md`](roborun-sdk/DEVELOPING.md) / [`roborun-station/DEVELOPING.md`](roborun-station/DEVELOPING.md)
for changing the packages themselves. What changed in each release: [`roborun-sdk/CHANGELOG.md`](roborun-sdk/CHANGELOG.md)
(`roborun docs changelog`).

## Examples

[`roborun-sdk/examples/`](roborun-sdk/examples/): a minimal arm driver, a camera from a folder of images, a keep-out rules
file, a command-line client for the local API, the robot as an LLM's tools, and a rover of your own (`rover.py`, a custom
robot body). `roborun new my-robot` writes a starter
project.

## Tests

```sh
pip install "./roborun-station" "./roborun-sdk[robot,test]" pytest-xdist pytest-timeout
cd roborun-sdk && python -m pytest -q -n auto --timeout=180 \
  --ignore=tests/test_guided_setup.py
cd ../roborun-station && python -m pytest -q
```

This is the selection [the CI workflow](.github/workflows/ci.yml) runs: simulation and fakes only, no arm or camera
needed. Some tests skip themselves when an optional SDK or
hardware is absent. `tests/test_guided_setup.py` runs separately: it needs root inside a network namespace with the
`vcan` kernel module and the network (it installs packages from PyPI); see
[`roborun-sdk/DEVELOPING.md`](roborun-sdk/DEVELOPING.md#run-the-tests).

## Issues and security

Bugs and requests for new arm, camera or robot support: open an issue with the templates in this repository. Pull
requests are welcome, especially arm drivers, cameras, examples and docs: see [CONTRIBUTING.md](CONTRIBUTING.md). Security problems: do not open a public issue, see [SECURITY.md](SECURITY.md).

## License

Apache-2.0, see [LICENSE](LICENSE) and [NOTICE](NOTICE). You may use, change and redistribute this code, including in
commercial products, under the terms of that license. Contributions are accepted under the same license
([CONTRIBUTING.md](CONTRIBUTING.md)). Third-party components keep their own licenses:
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md). The license does not grant rights to the Roborun name or logo beyond
what its section 6 allows.

## Service terms

The Apache-2.0 license covers the code in this repository. The hosted Roborun service (app.roborun.app and its API, your
Roborun key) and the robot runtime that `roborun up` downloads from it are not in this repository and are covered by
separate terms: <https://roborun.app/terms>.
