<p align="center">
  <img src="assets/demo-thumb.jpg" alt="roborun" width="100%">
</p>

<h1 align="center">roborun</h1>

<p align="center"><b>Your AI drives any ROS robot in 5 minutes.<br>Every run is a replayable black box.</b></p>

<p align="center">
  <a href="https://pypi.org/project/ros-agent/"><img src="https://img.shields.io/pypi/v/ros-agent?style=flat-square&color=00d47e&label=pip%20install%20ros-agent" alt="PyPI"></a>
  <a href="https://python.org"><img src="https://img.shields.io/badge/python-3.10+-3776AB?style=flat-square" alt="Python 3.10+"></a>
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-MIT-green?style=flat-square" alt="MIT"></a>
  <a href="https://modelcontextprotocol.io"><img src="https://img.shields.io/badge/MCP-native-00d47e?style=flat-square" alt="MCP native"></a>
</p>

---

## 30 seconds

```bash
pip install ros-agent
ros-agent
```

Browser opens. No robot? Your webcam starts with live object detection. Have a robot? Point it at the IP — ROS 1 or ROS 2 over rosbridge, **no ROS install needed on your machine**.

Then tell Claude (or Cursor, or any MCP client) to drive it:

```json
{ "mcpServers": { "ros-agent": { "command": "ros-agent-mcp" } } }
```

One line of config. Your AI can now see camera feeds, read sensors, and move the robot.

## The black box

Everything the robot saw, decided, and did is recorded into one event timeline — operator commands, agent tool calls, velocity commands, detections. Seal it and it becomes evidence:

```bash
python -m roborun.integrity seal   runs/run_20260609_153000
# SEALED — 1,284 events
# merkle root: 8f4a2c91…  signed: ed25519

python -m roborun.integrity verify runs/run_20260609_153000
# VERIFIED — 1,284 events, signature valid
```

Now edit one byte of the log:

```bash
python -m roborun.integrity tamper runs/run_20260609_153000 --event 42
python -m roborun.integrity verify runs/run_20260609_153000
# FAILED — event 0042 hash mismatch
# expected: sha256:91ac3be0…
# found:    sha256:338e7d12…
```

One changed byte, caught instantly. Same primitives as Git and Certificate Transparency: SHA-256 Merkle tree + Ed25519 signature. No cloud, no vendor, works offline.

## The flight deck

`http://localhost:8765/demo` — a mission-control view built for watching (and recording) your robot work: live camera with detections, the black box streaming in real time, and a command bar wired to the agent.

Press `S` to seal the run, `T` to tamper one byte, `V` to verify. Watch it fail.

## Connect a robot

```bash
# on the robot (ROS 2)
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# on the robot (ROS 1)
roslaunch rosbridge_server rosbridge_websocket.launch
```

Set the IP in the UI, or `curl -X POST localhost:8765/api/ros/connect -d '{"host":"192.168.1.100"}'`. Works with real hardware (Unitree Go2/G1, TurtleBot, arms, drones), NVIDIA Isaac Sim, Gazebo, or the built-in MuJoCo sim.

## What your AI gets

MCP tools for topic discovery, pub/sub, service calls, camera snapshots, depth, velocity control, and full graph introspection — plus skills (patrol, follow-me, object search) that load as plugins:

```python
SKILL_TOOLS = [{"name": "my_tool", "description": "Does a thing",
                "inputSchema": {"type": "object", "properties": {}}}]

def handle(name: str, args: dict) -> str:
    return "done"
```

Drop the file in a directory, point `ROBORUN_SKILL_PATHS` at it. It becomes an MCP tool.

Optional extras: `pip install ros-agent[vision]` (YOLO + CLIP), `[sim]` (MuJoCo), `[ros]` (direct DDS), `[all]`.

## Configuration

| Variable | Default | |
|----------|---------|---|
| `ROBORUN_PORT` | `8765` | Server port |
| `ROBOT_IP` | — | Robot IP (or set in UI) |
| `ANTHROPIC_API_KEY` | — | Enables the built-in Claude agent |
| `ROBORUN_MAX_LINEAR_VEL` | `1.0` | Safety clamp, m/s |
| `ROBORUN_MAX_ANGULAR_VEL` | `1.5` | Safety clamp, rad/s |

## Why

Everyone is wiring LLMs to robots. Nobody can prove what the robot actually did. roborun does both: natural-language control on the way in, tamper-evident evidence on the way out. When your robot does something weird at 3am, you replay the run — and you can prove nobody edited it.

## Contributing

```bash
git clone https://github.com/publu/RoboRun.git && cd RoboRun
pip install -e ".[all]"
python -m roborun.server
```

MIT — built by [Hashing Systems](https://hashingsystems.com).
