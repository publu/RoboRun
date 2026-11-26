# Roborun SDK

**Turn plain-English instructions and camera views into verified motion on any robot arm or body — at a fraction of the cost of driving robots directly with frontier VLMs.**

Roborun splits robot control cleanly into two halves:

- **Local on your machine (this repo, Apache-2.0):** arm drivers, cameras, custom body adapters, MuJoCo simulation, local keep-out/clearance safety rules, and an unrefusable `Stop`.
- **Hosted Roborun API:** motion planning from your camera view, robot profile, and instruction — returning verified trajectories while keeping hardware control and safety local.

> **Using an AI coding assistant?** Point it at [`roborun-sdk/docs/AGENTS.md`](roborun-sdk/docs/AGENTS.md) (or run `roborun docs agents` after install) for a single-page setup, drive, and stop reference.

---

## Why Roborun? Cost & Benchmark Data

Using a frontier VLM directly as a low-level robot controller works, but burns dozens of reasoning turns per task — typically **$1–$16+ per episode** and **20–30 minutes** on real or simulated arms — while smaller, low-cost VLMs fail completely at low-level pose control.

Because the Roborun API handles physical motion planning in a single high-level task call (`POST /v1/tasks`, with optional pixel `marks` from any camera), **even a $0.10/M-token VLM (`gpt-6-luna`) achieves 90%+ zero-shot pick-and-place success**, cutting end-to-end cost per task by **10×–100×** and wall time by **~15×**.

### Measured Cost & Performance (Simulated Franka Panda, Zero-Shot)

| Setup | Benchmark | Success Rate | API Calls / Tasks per Ep | Wall Time | VLM Token Cost / Ep\* |
| --- | --- | ---: | ---: | ---: | ---: |
| **GPT-6 Astra (high)** direct† | LIBERO-10 | 44% (44/100) | 49.4 pose tool calls | — | **$7.09** ($16.11 / pass) |
| **GPT-6.1 Sol (high)** direct† | LIBERO-10 | 33% (33/100) | 53.8 pose tool calls | — | **$1.64** ($4.97 / pass) |
| **GPT-6.1 Sol (low)** direct† | LIBERO-10 | 24% (24/100) | 51.6 pose tool calls | — | **$1.02** ($4.25 / pass) |
| **GPT-6 Luna (all efforts)** direct† | LIBERO-10 | **0%** (0/300) | 8.5–74.0 pose tool calls | — | $0.003–$0.29 (no passes) |
| **`gpt-6-luna` + Roborun API** | LIBERO-Object | **90%** (9/10) | **1.3 `POST /v1/tasks` calls** | ~2.1 min | **$0.013** |
| **`gpt-6.2-sol` (low) + Roborun API** | LIBERO-Object | **100%** (10/10) | **1.0 `POST /v1/tasks` call** | ~1.8 min | **$0.194** |
| **`gpt-6.1-sol` (high) + Roborun API** | LIBERO-Object (5 layouts × 10) | **88%** ≤600 steps (44/50; **98%** any step) | **1.6 `POST /v1/tasks` calls** | ~2.4 min | **$0.187** |

*\*VLM token cost per episode measured from logged input/cached/output tokens at published API rates, plus the Roborun task calls shown (`POST /v1/tasks`).*  
*†Direct-VLM baselines from Espinola, Zhang & Xiang, [arXiv:2610.30051](https://arxiv.org/abs/2610.30051) (Oct 2026), evaluating frontier models issuing closed-loop end-effector pose commands on LIBERO-10 without pretrained manipulation skills.*

**Evaluation notes:** All Roborun trials above run zero-shot (no task-specific fine-tuning) through the standard Roborun local API and hosted service with LIBERO's own BDDL success predicates and 600-step budget. LIBERO-Object runs use a 960×720 scene camera, frozen physics during planning, an open start gripper, and a one-page product reference sheet; multi-suite zero-shot evaluations (LIBERO-10, LIBERO-Spatial, and LIBERO-Goal without reference sheets) are in progress and will be added to this table as runs complete.

---

## Install

**Official installer** (macOS and Linux; Windows via WSL2 — installs its own Python, no admin rights required):

```sh
curl -fsSL https://roborun.app/install.sh | sh
```

**From source** (Python 3.10+):

```sh
git clone https://github.com/publu/RoboRun.git roborun-sdk && cd roborun-sdk
python -m venv .venv && . .venv/bin/activate
pip install ./roborun-station "./roborun-sdk[robot]"
```

Or install straight from GitHub (`roborun-station` first):

```sh
pip install "git+https://github.com/publu/RoboRun.git#subdirectory=roborun-station" \
            "roborun-sdk[robot] @ git+https://github.com/publu/RoboRun.git#subdirectory=roborun-sdk"
```

- **`roborun-sdk[robot]`** includes everything a robot runs (simulator, cameras, arm drivers) — matching the official installer.
- **`roborun-sdk`** (light install) installs only the Python client, driver/rule interfaces, and `roborun` CLI with no simulator or hardware dependencies. You can also select specific extras such as `pip install "./roborun-sdk[cameras,feetech]"` (see [`roborun-sdk/pyproject.toml`](roborun-sdk/pyproject.toml) and [Install only what you need](roborun-sdk/docs/BUILDING.md#install-only-what-you-need)).

Using the hosted Roborun API requires a key. Access is currently invite-only — email **support@roborun.dev** to request one.

---

## 60-Second Quickstart (Simulation)

```sh
roborun login                  # opens a sign-in link on https://app.roborun.app
roborun up --sim cup           # starts two simulated arms + a cup locally (downloads runtime on first run)
curl -X POST http://127.0.0.1:8120/v1/tasks \
  -H 'Content-Type: application/json' \
  -d '{"prompt": "pick up the cup"}'
roborun stop                   # Stop is local and never refused
roborun down
```

Or open <https://app.roborun.app> on the same computer to send tasks and watch the live cameras. See [`roborun-sdk/README.md`](roborun-sdk/README.md) for the full walkthrough.

### Drive from Python or a VLM

```python
from roborun_sdk.client import Robot

robot = Robot()
cams = robot.cameras()                       # pixel size, mount, intrinsics, pose for every camera
task = robot.run(
    "pick up the milk and place it in the basket",
    camera="top",                            # or "left" / "right" for wrist-camera tasks
    marks={                                  # optional pixel boxes in the scene image
        "camera": "top",
        "target": [269, 279, 326, 383],
        "place": [625, 297, 810, 470],
    },
)
print(task["status"], task["message"], task["outcome"])
```

For ready-to-use LLM tool schemas (`cameras`, `frame`, `run_task`, `robot_state`, `stop`), see [`roborun-sdk/examples/llm_tools.py`](roborun-sdk/examples/llm_tools.py).

### Try Locally Without a Key

These examples run completely offline on your machine (from `roborun-sdk/examples/`):

```sh
python minimal_arm.py          # smallest arm driver on a local station
python folder_camera.py DIR    # custom camera backed by a folder of images
python keep_out_box.py         # local safety rule keeping the gripper out of a box
roborun new my-robot           # scaffold a starter project (driver, rules, task client)
roborun docs                   # browse guides locally (`roborun docs --examples DIR` copies examples)
```

---

## Built for Any Embodiment

Roborun is designed so you can wire in your own arms, cameras, and robot bodies without modifying the hosted service:

- **Custom arms & URDFs:** wrap any joint-controlled arm via [`Your own arm driver`](roborun-sdk/docs/BUILDING.md#your-own-arm-driver) or generate a profile from a URDF ([`Arm profile from a URDF`](roborun-sdk/docs/BUILDING.md#arm-profile-from-a-urdf)).
- **Custom cameras & camera cards:** attach any overhead, tripod, or wrist camera with intrinsics, pose, mount `description`, and wrist `fingers` metadata ([`Describe your cameras`](roborun-sdk/docs/BUILDING.md#describe-your-cameras)).
- **Custom robot bodies:** support for wheeled, tracked, legged/quadruped, and humanoid embodiments (`0–4` arms) — see [`Your own robot body`](roborun-sdk/docs/BUILDING.md#your-own-robot-body) and [`roborun-sdk/examples/rover.py`](roborun-sdk/examples/rover.py), plus built-in profiles for [Booster K1 and Galaxea R1](roborun-sdk/docs/BUILDING.md#humanoids-booster-k1-and-galaxea-r1).
- **Local safety rules:** keep-out boxes, velocity/clearance limits, and `<60 ms` Stop run strictly on your robot computer ([`Rules`](roborun-sdk/docs/BUILDING.md#rules)).

---

## Repository Layout

| Path | Description |
| --- | --- |
| [`roborun-sdk/`](roborun-sdk/) | `roborun` CLI and `roborun_sdk` package: guided setup, arm drivers, cameras, MuJoCo simulation, local API, and Python client |
| [`roborun-station/`](roborun-station/) | Pure-Python station hosting your arm drivers: motion leases, Stop and hold, Home, and simulated stations |
| [`roborun-sdk/examples/`](roborun-sdk/examples/) | Minimal arm driver, folder camera, keep-out rules, CLI client, VLM tool schemas (`llm_tools.py`), and custom rover (`rover.py`) |
| [`roborun-sdk/docs/BUILDING.md`](roborun-sdk/docs/BUILDING.md) | Complete builder reference (drivers, cameras, rules, URDFs, local API, client, LLM tools, humanoids, custom bodies) |
| [`roborun-sdk/docs/HOSTED_API.md`](roborun-sdk/docs/HOSTED_API.md) | Hosted Roborun API reference |
| [`roborun-sdk/CHANGELOG.md`](roborun-sdk/CHANGELOG.md) | Release notes (`roborun docs changelog`) |

The robot runtime is not stored in this repository; `roborun up` downloads it automatically on first start. Earlier `ros-agent` releases (v0.1.0–v0.9.0) are archived on the [`legacy`](https://github.com/publu/RoboRun/tree/legacy) branch (`legacy-v0.1.0`..`legacy-v0.9.0`, `legacy-feat-fleet-lab`, `legacy-openclaw-mcp`).

---

## Running Tests

```sh
pip install "./roborun-station" "./roborun-sdk[robot,test]" pytest-xdist pytest-timeout
cd roborun-sdk && python -m pytest -q -n auto --timeout=180 \
  --ignore=tests/test_guided_setup.py
cd ../roborun-station && python -m pytest -q
```

This matches [`.github/workflows/ci.yml`](.github/workflows/ci.yml) (simulation and fakes only; no hardware required). For package development and `test_guided_setup.py`, see [`roborun-sdk/DEVELOPING.md`](roborun-sdk/DEVELOPING.md) and [`roborun-station/DEVELOPING.md`](roborun-station/DEVELOPING.md).

## Issues, Security & License

- **Bugs, drivers, and PRs:** open an issue or pull request — see [CONTRIBUTING.md](CONTRIBUTING.md).
- **Security issues:** do not open a public issue; follow [SECURITY.md](SECURITY.md).
- **Code license:** Apache-2.0 ([LICENSE](LICENSE), [NOTICE](NOTICE), [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)).
- **Service terms:** the hosted Roborun service (`app.roborun.app`, its API, and the downloaded robot runtime) is covered by <https://roborun.app/terms>.
