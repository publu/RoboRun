# Roborun

Tell your robot arms what to do. Manifest turns your words and your camera view into motion, and Roborun runs it on your
arms, with safety and Stop on your own computer.

**Using an AI assistant?** Have it read `roborun docs agents` (also `~/.roborun/AGENTS.md` after install): one short file
with what it needs to install, start, drive and stop Roborun for you.

## 1. Install

```sh
curl -fsSL https://roborun.app/install.sh | sh
```

macOS and Linux (Windows: WSL2), no administrator rights, its own Python. If something needs one administrator command
(the simulator's OpenGL library, USB access for real arms), Roborun prints that exact line.

Only the Python SDK, in your own environment (the client, your own drivers and rules): `pip install roborun-sdk` is small
(no simulator, OpenCV or numpy), and `roborun-sdk[robot]` is everything a robot runs, as the installer installs it. See
docs/BUILDING.md, "Install only what you need".

## 2. Sign in

```sh
roborun login
```

It prints a link: open it, sign in on <https://app.roborun.app> and approve this computer. The next `roborun up` saves the
new key (`roborun login --wait` waits for the approval right there). Roborun is invite-only: if you don't have a key, email
support@roborun.dev.

## 3. Start a simulated robot

```sh
roborun up --sim cup
```

Two simulated YAM arms and a cup, on this computer. The first start downloads the robot runtime, so it takes a minute or two.
It ends with `ready: open https://app.roborun.app on this computer`. Your own arms instead: plug them in and run `roborun up`,
which walks you through a guided setup the first time.

## 4. Run a first task

Type a prompt on <https://app.roborun.app> (open it on this computer), or use the local API, which answers only on this
computer:

```sh
curl -X POST http://127.0.0.1:8120/v1/tasks -H 'Content-Type: application/json' -d '{"prompt": "pick up the cup"}'
```

```json
{"id": "e01ae6e8...", "status": "running", "mode": "run", "model": "manifest-1-beta", "arm": null, "prompt": "pick up the cup", "moves": 0,
 "message": "Working", "created": "2026-11-18T10:43:39+00:00", "elapsed_s": 1.2,
 "recording_url": null, "error": null}
```

Follow it with `GET http://127.0.0.1:8120/v1/tasks/{id}` until `status` is `done`, `stopped` or `failed`. From Python
(`roborun python my_script.py`):

```python
from roborun_sdk.client import Robot
task = Robot().run('pick up the cup')      # starts it and waits until it ends
print(task['status'], task['message'], task['outcome'])
```

## 5. Stop

The Stop button on the page, `roborun stop`, or `POST /v1/stop` on the local API stop all motion at any time and are never
refused; `GET /v1/stop/status` says when everything is still (`200` means the Stop was taken; `503` means it could not be
confirmed: support the arm or cut its power). `roborun down` stops Roborun and
leaves the arms safe.

## Where to go next

- `roborun --help` for the commands, `roborun status` for what is running, `roborun view` for this robot's own viewer.
- [docs/BUILDING.md](docs/BUILDING.md), the reference: [your own arms](docs/BUILDING.md#your-arms-setup-and-hardware-checks),
  [profiles and simulation](docs/BUILDING.md#profiles-and-simulation), [the local API](docs/BUILDING.md#the-local-api),
  [your own driver, camera and rules](docs/BUILDING.md#your-own-arm-driver), [your own robot body](docs/BUILDING.md#your-own-robot-body)
  (a mobile base or humanoid), [privacy and your data](docs/BUILDING.md#privacy-and-your-data),
  and [upgrading or removing Roborun](docs/BUILDING.md#install-upgrade-and-remove).
- [examples/](examples/): a client, an LLM's tools, a minimal arm, a camera, a rules file, a rover. `roborun docs` says where all of
  this is on your computer.
- `roborun docs changelog` (CHANGELOG.md): what each release changed. Read it before `roborun upgrade`.

## License

Apache-2.0, see [LICENSE](LICENSE) and [NOTICE](NOTICE). Third-party components keep their own licenses:
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md). The hosted Roborun service and the robot runtime that `roborun up`
downloads are covered by separate terms.
