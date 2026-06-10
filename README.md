<p align="center">
  <img src="assets/demo-thumb.jpg" alt="roborun" width="100%">
</p>

<h1 align="center">RoboRun: Write a Robot Behavior Once, Run It on Any ROS 1/2 Robot</h1>

<p align="center"><b>The base layer for coding robots: <code>see / move / ask</code> primitives, hot-reload Python behaviors,<br>the same file from webcam + MuJoCo to real hardware. MCP-native for AI agents, every run flight-recorded.</b></p>

<p align="center">
  <a href="https://pypi.org/project/ros-agent/"><img src="https://img.shields.io/pypi/v/ros-agent?style=flat-square&color=00d47e&label=pip%20install%20ros-agent" alt="PyPI"></a>
  <a href="https://python.org"><img src="https://img.shields.io/badge/python-3.10+-3776AB?style=flat-square" alt="Python 3.10+"></a>
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-MIT-green?style=flat-square" alt="MIT"></a>
  <a href="https://modelcontextprotocol.io"><img src="https://img.shields.io/badge/MCP-native-00d47e?style=flat-square" alt="MCP native"></a>
</p>

---

## 60 seconds, no robot required

```bash
pip install ros-agent
ros-agent
```

The browser opens live: your webcam becomes the robot's eyes (YOLO autostarts), MuJoCo becomes its body, and a `behaviors/` folder appears with its brain:

```python
# behaviors/follow_person.py (already running)
from roborun.behaviors import behavior

@behavior(hz=10)
def follow_person(robot):
    people = robot.see("person")
    if not people:
        return robot.stop()
    robot.move(
        forward=0.3 if people[0].h < 0.6 else 0.0,  # stop when close
        turn=-1.2 * (people[0].cx - 0.5),           # steer toward center
    )
```

Change `0.3` to `0.6`. Save. The robot speeds up **while it's running**. No restart, no build, no launch files, no framework to learn. That's the whole loop: see, think, move, in a file you can read in ten seconds.

## The robot handle

| Call | Does |
|---|---|
| `robot.see("person")` | live detections, normalized `.cx .cy .w .h .label .conf` |
| `robot.move(forward, strafe, turn)` | drives the sim or a real robot, always safety-clamped |
| `robot.ask("is the door open?", image=True)` | LLM with the camera frame. Anthropic API **or local Ollama** |
| `robot.say(...)` / `robot.log(...)` | speak into the event timeline |
| `robot.remember(k, v)` / `robot.recall(k)` | memory that survives restarts |
| `robot.state` | dict that survives across loop ticks |

`@behavior(hz=10)` for control loops, `@behavior(every=10.0)` for slow ones (LLM narration, patrol logic). Files hot-reload on save. Broken files report into the timeline and never crash the runtime: a behavior that throws keeps its slot and tells you why.

## LLMs, local or online

`robot.ask()` uses the Anthropic API when `ANTHROPIC_API_KEY` is set, or a local [Ollama](https://ollama.com) otherwise (`OLLAMA_MODEL=llama3.2` by default). The shipped `narrator.py` behavior asks a vision model what the robot is looking at, every 10 seconds, and says it into the timeline.

The whole robot is also an MCP server. One line and Claude or Cursor drives it directly:

```json
{ "mcpServers": { "ros-agent": { "command": "ros-agent-mcp" } } }
```

## The black box

Everything (your commands, behavior moves, agent tool calls, detections, camera-frame hashes) journals to disk as it happens, and every event carries the SHA-256 of the previous one. The log is a hash chain: tamper-evident **while it's being written**, not just after. Seal it and it becomes evidence:

```bash
python -m roborun.integrity seal   ~/.roborun/runs/run_20260609_153000
# SEALED: 1,284 events
# merkle root: 8f4a2c91...
# signed: ed25519

python -m roborun.integrity tamper ~/.roborun/runs/run_20260609_153000 --event 42
python -m roborun.integrity verify ~/.roborun/runs/run_20260609_153000
# FAILED: event 0042 hash mismatch
```

One changed byte, caught instantly, and the exact event named. Hash chain + SHA-256 Merkle tree + Ed25519 signature: the same primitives as Git and Certificate Transparency. The merkle root is 64 characters. Share it anywhere (an email, a ticket, a printout) and anyone holding it can later prove the run wasn't quietly edited and resealed. No cloud, works offline, MIT licensed.

Sealed runs chain to each other: each new run's manifest records the previous run's merkle root. When your robot does something weird at 3am, you **replay the run in the UI** and you can prove nobody edited it.

### MCAP recordings — the black box, now with the evidence inside

Press `M` in the flight deck (or `POST /api/run/record/start`) and everything — camera keyframes, YOLO detections, CLIP embeddings, agent events, pose — records into **one MCAP file**, the same container Foxglove Studio replays natively. The hash chain moves to chunk granularity in a sidecar, the seal is O(1) (a Merkle root over chunk hashes, signed ed25519), and on seal the root is **anchored to an RFC 3161 trusted timestamp authority** — the same mechanism behind code signing — so the proof references an external clock, not our word:

```bash
python -m roborun.recorder verify ~/.roborun/runs/local/run_20260610_120000.mcap
# VERIFIED + ANCHORED: unchanged since trusted timestamp 2026-06-10T12:01:07Z (RFC 3161)
python -m roborun.recorder clip <run.mcap> <start_ts> <end_ts>
# cuts a window + a signed proof binding those exact frames to the sealed run
```

Verify is three-state, not binary: `verified + anchored`, `internally consistent (unanchored)` — e.g. a robot that was offline; it anchors when connectivity returns — or `broken`, naming the exact chunk and byte range. Tap mode (the `telemetry_stream` MCP tool) records ROS topics into the run at full rate with no LLM in the loop, over DDS direct (common message families, vendored in `roborun.transport`) or rosbridge.

On run close, the MCAP is extracted into a local SQLite index (indexed label search, CLIP cosine, spatial queries) and optionally exported as Parquet to R2, where **embedded DuckDB queries the whole fleet** — `search_clip("red mug")` across every robot — and robots share Ed25519-signed beacons through the same bucket. Local files and R2 only: no brokers, no database servers, nothing to operate.

What this proves: the recorded run — images, detections, and decisions included — hasn't been altered since a moment an external clock witnessed. What it doesn't prove: that the robot's sensors observed reality correctly. We're precise about this distinction on purpose.

The UI at `http://localhost:8765` is the flight deck itself: live camera with YOLO boxes, the black box streaming, the live anchor badge, a command bar, and director keys. `S` seal · `V` verify · `T` tamper · `M` record mcap · `R` runs/replay · `C` sources.

## Connect a real robot

```bash
# on the robot
ros2 launch rosbridge_server rosbridge_websocket_launch.xml   # ROS 2
roslaunch rosbridge_server rosbridge_websocket.launch          # ROS 1
```

Point the UI at the robot's IP. **No ROS install on your machine.** The same `behaviors/*.py` files now drive real hardware: Unitree Go2/G1, TurtleBot, arms, drones, NVIDIA Isaac Sim, Gazebo. `robot.move()` goes to the sim if it's running, otherwise to the connected robot, always through the same safety clamps.

Optional extras: `pip install ros-agent[vision]` (YOLO + CLIP), `[sim]` (MuJoCo), `[ros]` (direct DDS), `[crypto]` (Ed25519 signing), `[anchor]` (RFC 3161 timestamping), `[fleet]` (R2 + DuckDB cross-robot), `[all]`.

## Configuration

| Variable | Default | |
|----------|---------|---|
| `ROBORUN_PORT` | `8765` | Server port |
| `ROBOT_IP` | unset | Robot IP (or set in UI) |
| `ANTHROPIC_API_KEY` | unset | `robot.ask()` + built-in Claude agent |
| `OLLAMA_MODEL` | `llama3.2` | Local model for `robot.ask()` |
| `ROBORUN_BEHAVIOR_PATHS` | unset | Extra behavior directories (comma-separated) |
| `ROBORUN_AUTOSTART` | `1` | Autostart camera/sim on boot |
| `ROBORUN_MAX_LINEAR_VEL` | `1.0` | Safety clamp, m/s |
| `ROBORUN_MAX_ANGULAR_VEL` | `1.5` | Safety clamp, rad/s |

## Why this instead of a robot framework

Robot frameworks make you learn their world first: module systems, typed streams, blueprints, launch graphs, all before the robot does anything. roborun inverts it. The robot is already running, and you change its mind by saving a file. Python you already know, hot-reloaded, with vision, an LLM, and motion in one handle, plus a cryptographic record of everything it did.

## Contributing

```bash
git clone https://github.com/publu/RoboRun.git && cd RoboRun
pip install -e ".[all]"
python -m roborun.server
pytest tests/
```

MIT. Built by Manifest Intelligence, Inc.
