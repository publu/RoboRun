# Roborun: the guide for an AI assistant

You are running Roborun for a person: robot arms (or a humanoid) on the computer they are wired to, driven by plain prompts. Get this file with `roborun docs agents` (also `~/.roborun/AGENTS.md` and <https://roborun.app/agents.md>). The full reference is `roborun docs building` ([BUILDING.md](BUILDING.md)); the links below go to its sections.

**Safety first.** Stop is always available and never refused: `roborun stop`, `POST http://127.0.0.1:8120/v1/stop`, or the Stop button on either page. Call it on any doubt. Never judge whether an arm holds something from its gripper opening: read the task's `outcome` and, when it says `"unknown"`, look at a camera first. Never ask the person to move real arms by hand unless a message tells you to.

## 1. Install and sign in

1. `curl -fsSL https://roborun.app/install.sh | sh` (macOS, Linux; Windows: WSL2). No admin rights; when one admin command is needed (the simulator's OpenGL library: `sudo apt install libosmesa6`; USB access for real arms), Roborun prints that exact line for the person to run. [More](BUILDING.md#install-upgrade-and-remove)
2. `roborun login` prints `Open https://app.roborun.app/device?code=XXXX-XXXX and approve this computer.` and exits: give the person that link. The next `roborun up` waits for the approval and saves the key (`roborun login --wait` waits right there). Never ask for a key in the chat: a key they already have goes in a file of theirs (`chmod 600`), passed once with `roborun up --key-file FILE`, or in `ROBORUN_API_KEY`. Invite-only: no key, email support@roborun.dev. [More](BUILDING.md#sign-in-and-keys)
3. `roborun up --sim cup` (no arms), or `roborun up` (their arms). A good start ends with `ready: open https://app.roborun.app ...` and `local API: http://127.0.0.1:8120/v1/docs`. The first start downloads the runtime and sets up recording once (about 290 MB, LeRobot datasets in `~/roborun/datasets/<robot>/`; skip with `roborun up --no-dataset`; if it fails, the robot still runs and recordings say `recorded: false`).

## 2. Start, check, stop

| Do | Command |
| --- | --- |
| Simulated arms (two YAMs) | `roborun up --sim cup` (scenes `cup`, `pickup-bench`, `mixed-objects`, `cluttered-bench`, `low-light`) |
| Their arms, first time (a guided setup; needs a terminal) | `roborun up`; later `roborun setup`; a read-only look: `roborun hw detect` |
| A simulated humanoid | `roborun up --profile galaxea_r1 --sim room` (or `booster_k1`; scenes `room`, `empty`) |
| Is it healthy; back to real arms | `roborun status`; `roborun up --no-sim` |
| Stop motion now / stop Roborun | `roborun stop` / `roborun down` (a held arm stays held: `roborun down --park`, or `--release` while supported) |
| Upgrade / remove | `roborun upgrade` / `roborun uninstall --yes` (`--purge` also deletes settings, key, recordings and datasets) |
| Logs | `journalctl --user -u roborun-robot` (also `roborun-sim`, `roborun-station`, `roborun-local`); no systemd: `~/.config/roborun/logs/` |

Without a terminal and with no arms set up, `roborun up` starts nothing and prints the manual steps (exit 1). If it prints `note: Roborun stops when you log out`, ask the person to run the admin line it gives. [More](BUILDING.md#running-roborun)

## 3. Profiles

`roborun profiles` lists them (`*` = running); switch with `roborun up --profile NAME`. `yam` is one YAM arm, `yam_dual` two, `yam_metal` a YAM and a Metal arm; any catalog arm works as `<arm>`, `<arm>_dual` or `<left>+<right>`; humanoids are `booster_k1` and `galaxea_r1`. Your own arm: `roborun profiles create --urdf ...`. Your own mobile base or humanoid: a profile in `~/.config/roborun/bodies/<id>.yaml` and `roborun up --profile <id> --body my_body.py:MyBody` ([Your own robot body](BUILDING.md#your-own-robot-body)). [More](BUILDING.md#profiles-and-simulation)

## 4. Real arms

The guided setup does every step and asks before anything moves. One at a time: `roborun hw detect` (adapters, cameras, likely arms; moves nothing), `roborun hw check ARM@ADDRESS` (reads, enables nothing), `roborun hw calibrate ARM@ADDRESS`, then `roborun hw check ARM@ADDRESS --move` (in a terminal, with the person). `roborun cameras describe --json` says where each camera is mounted, what it delivers and which task camera (`top`, `left`, `right`) it answers as; it opens nothing. [More](BUILDING.md#your-arms-setup-and-hardware-checks)

## 5. The pages

<https://app.roborun.app> (opened **on the robot's computer**: prompt, cameras, 3-D view, Stop, Home) and `roborun view` (`http://127.0.0.1:8135/`: this robot's own read-only viewer, no sign-in). [More](BUILDING.md#the-viewer-and-the-page)

## 6. The local API (`http://127.0.0.1:8120`, this computer only, no key)

Arms: `POST /v1/tasks` (`{"prompt": "pick up the cup"}`; optional `mode` `run` or `preview`, `arm` `left`, `right`, `both`), `GET /v1/tasks/{id}` (and `/live`, `/recording`), `POST /v1/tasks/{id}/stop` (a policy run from `roborun policy run`: stop it with the Python client's `stop_task(id)` or `POST /v1/stop`), `POST /v1/stop`, `GET /v1/stop/status`, `GET /v1/cameras/{role}.jpg` (`top`, `left`, `right`; faces blurred), `GET /v1/robot`, `GET /v1/status`, `GET /v1/docs`. Humanoids: `POST /v1/body/move` (`{"prompt": "drive to the table", "duration_s": 4}`), `GET /v1/body/moves/{id}`, `GET /v1/body/state`, `GET /v1/cameras/head.jpg` (not face-blurred yet), and the same Stop and status routes. [More](BUILDING.md#the-local-api)

A task's `status` is `running`, `done`, `stopped` or `failed`, with `message` and, when finished, `outcome`: `holding.left` / `holding.right` is `{object, lifted_mm, evidence, verified}`, `null` (measured empty) or `"unknown"` (not checked: look first). After "pick up the cup" read `outcome.holding.left`; after "put it down" it is `null`.

Python (`roborun python script.py`): `from roborun_sdk.client import Robot; task = Robot().run('pick up the cup')`. Methods: `status()`, `health()`, `state()`, `frame(camera)`, `start(prompt, mode, arm)`, `task(id)`, `tasks(limit)`, `live(id)`, `wait(id, timeout, on_update)`, `run(...)`, `stop_task(id)`, `recording(id)`, `body_move(prompt, duration_s)`, `body_move_status(id)`, `body_run(...)`, `stop()`, `stop_status()`, `call(method, path, body)`.

## 7. Errors and what to do

Every local error is `{"error": {"code", "message", "retryable"}}`; `retryable: true` means the same call may work later. [More](BUILDING.md#errors)

| Code | Next |
| --- | --- |
| `not_running`, `unreachable` | Nothing answers (Python client: status 0), or the arms do not: `roborun status`, logs, `roborun down` then `roborun up` |
| `invalid`, `not_found` | Fix the request; `/v1/docs` lists the routes |
| `busy` | Another task or move runs: wait, or Stop it |
| `unavailable`, `blur_unavailable` | No fresh (blurred) frame yet: retry in a second |
| `forbidden`, `method_not_allowed` | A page-only route or the wrong method: use the `/v1` routes |
| `stop_not_confirmed` | Tell the person: support the arm or cut its power |
| `internal`, `robot_error` | Logs, then `roborun down` and `roborun up`; a real body: check it and its agent |
| `no_motion_service` | A humanoid with no key: `roborun login`, then `roborun up` |

A `failed` task carries `error_info: {code, retryable, retry_after_s, next}`: follow `next`. Codes: `network_offline`, `service_unavailable`, `service_timeout`, `rate_limited` (retry later), `key_revoked` (`roborun login`), `signing_key_unknown` (`roborun up`), `sdk_upgrade_required` (`roborun upgrade`), `object_not_measured` (the top camera must see the object), `no_depth_camera` (tasks need a depth camera; a webcam only shows and records), `task_failed` (check the robot is clear and powered, then retry). [More](BUILDING.md#when-a-task-fails)

## 8. The person's data

Frames that leave the computer are face-blurred first, except a humanoid's head frame, not blurred yet: each move sends it to the hosted API as it is, and `GET /v1/cameras/head.jpg` gives it sharp, so don't send it on; history (task records, recordings, and hourly robot telemetry: joints, jaws, speeds, temperatures and events like Home, no images) uploads to the person's account unless `roborun up --no-data-sync`. Each robot's history is a LeRobot v2.1 dataset, read with their key (`Authorization: Bearer KEY`; `{robot}` is the name `roborun status` shows): `GET https://api.roborun.app/v1/robots/{robot}/episodes` (episodes: prompt, length, outcome, when), `GET https://api.roborun.app/v1/robots/{robot}/dataset.tar` (unpack it; load `dataset/` with LeRobot), and `POST https://api.roborun.app/v1/robots/{robot}/data/delete` with `{"confirm": "<robot>"}`, which deletes everything kept for that robot and cannot be undone: only when the person asks for exactly that, naming the robot back to them first. [More](BUILDING.md#privacy-and-your-data)
