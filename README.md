<p align="center">
  <img src="assets/demo-thumb.jpg" alt="RoboRun" width="100%">
</p>

# RoboRun — ros-agent

### `pip install ros-agent` — let any AI control any robot

<p align="center">
  <a href="https://pypi.org/project/ros-agent/"><img src="https://img.shields.io/pypi/v/ros-agent?style=for-the-badge&color=00d47e&label=PyPI" alt="PyPI"></a>
  <a href="https://python.org"><img src="https://img.shields.io/badge/Python-3.10+-3776AB?style=for-the-badge&logo=python&logoColor=white" alt="Python 3.10+"></a>
  <a href="LICENSE"><img src="https://img.shields.io/badge/License-MIT-green?style=for-the-badge" alt="MIT License"></a>
  <a href="https://modelcontextprotocol.io"><img src="https://img.shields.io/badge/MCP_Tools-49-00d47e?style=for-the-badge" alt="49 MCP Tools"></a>
  <a href="https://ros.org"><img src="https://img.shields.io/badge/ROS-1_&_2-22314E?style=for-the-badge&logo=ros&logoColor=white" alt="ROS 1 and ROS 2"></a>
</p>

**Open-source AI agent framework for robotics.** 49 [MCP](https://modelcontextprotocol.io) tools that let [Claude](https://claude.ai), [Cursor](https://cursor.com), or any AI control a real robot — ROS 1, ROS 2, NVIDIA Isaac Sim, or just a webcam. Plugin skills for autonomous behaviors (patrol, follow-me, object search). Real-time computer vision (YOLO, CLIP, JEPA). MuJoCo physics simulation. Multi-robot fleet management. Two pip dependencies — no ROS install required on your machine.

Works with any MCP-compatible AI client: [Claude Desktop](https://claude.ai), [Claude Code](https://claude.ai/code), [Cursor](https://cursor.com), [Windsurf](https://codeium.com/windsurf), or your own. Add one line to your config and your AI gets camera feeds, velocity control, sensor data, autonomous patrol, person following, object search, and full ROS introspection. Connects over rosbridge (ROS 1 + ROS 2) or direct DDS (ROS 2). Switch robots by changing an IP address.

<table>
<tr><td><b>MCP-native from the ground up</b></td><td>49 tools, 8 guided prompts, 6 live resources, and a topic template — all exposed through standard MCP. HTTP+SSE and stdio transports. Works with any client that speaks the protocol.</td></tr>
<tr><td><b>Skills plugin system</b></td><td>5 built-in skills (compose, inspect, follow-me, patrol, scan-detect). Write your own in 10 lines of Python. 4 loading paths: built-in, pip packages, filesystem, project config. Every skill becomes MCP tools automatically.</td></tr>
<tr><td><b>Real-time vision stack</b></td><td>YOLO object detection + tracking, CLIP zero-shot search ("find the red cup"), JEPA self-supervised attention heatmaps, Cosmos 3 world model (16B, MLX 4-bit on Mac). Toggle models live from the UI.</td></tr>
<tr><td><b>Any robot, no ROS install needed</b></td><td>Connects over rosbridge WebSocket (ROS 1 + ROS 2) or direct DDS (ROS 2) — no ROS installation on the host. Works with real hardware (Unitree Go2/G1, TurtleBot, drones, arms), NVIDIA Isaac Sim, Gazebo, or just a webcam.</td></tr>
<tr><td><b>Built-in Claude + Gemini agent</b></td><td>Streaming tool use with dynamic ROS context injection. Safety velocity clamping. Persistent cross-session memory. Behavioral identity via SOUL.md.</td></tr>
<tr><td><b>Simulation and fleet</b></td><td>Built-in MuJoCo headless physics with ONNX locomotion policies. Also connects to NVIDIA Isaac Sim and Gazebo via rosbridge. Fleet dashboard for multi-robot management. Blueprint system. CLIP-indexed spatial memory with geo-search.</td></tr>
</table>

---

## Quick Install

```bash
pip install ros-agent
ros-agent
```

That's it. Browser opens at `http://127.0.0.1:8765`. Webcam starts with live detection.

### From source

```bash
git clone https://github.com/publu/RoboRun.git
cd RoboRun
pip install -e .
ros-agent
```

### npx (Node 18+)

```bash
npx ros-agent
```

### Optional extras

```bash
pip install ros-agent[vision]   # YOLO + CLIP + OpenCV
pip install ros-agent[sim]      # MuJoCo physics simulation
pip install ros-agent[ros]      # Direct DDS (CycloneDDS)
pip install ros-agent[gemini]   # Gemini agent
pip install ros-agent[all]      # Everything
```

---

## Connect Your AI

### Claude Desktop / Cursor (HTTP)

```json
{
  "mcpServers": {
    "ros-agent": {
      "type": "http",
      "url": "http://localhost:8765/mcp"
    }
  }
}
```

### Claude Code / CLI clients (stdio)

```json
{
  "mcpServers": {
    "ros-agent": {
      "command": "ros-agent-mcp"
    }
  }
}
```

Your AI immediately gets 49 tools for robot control, 8 guided prompts for common workflows, and 6 live resources for introspection.

---

## What Your AI Gets

### 49 Tools

30 built-in ROS tools (topic discovery, pub/sub, service calls, action goals, parameter management, camera snapshots, depth images, velocity commands, node/topic/service/action introspection) plus 19 skill tools from 5 built-in skills.

### 8 Prompts

| Prompt | What it does |
|--------|-------------|
| `explore-robot` | Guided discovery of a robot's capabilities |
| `safety-check` | Pre-operation safety verification |
| `environment-scan` | Full environment survey with camera + sensors |
| `teach-waypoints` | Interactive waypoint teaching for patrol |
| `debug-topic` | Step-by-step topic diagnosis |
| `quick-start` | First-time onboarding in 60 seconds |
| `fleet-sweep` | Multi-robot status check |
| `build-workflow` | Create a reusable tool chain |

### 6 Resources

| Resource | Description |
|----------|-------------|
| `ros-agent://server-info` | Server version, uptime, capabilities |
| `ros-agent://skills` | Loaded skills and their tools |
| `ros-agent://ros-graph` | Live ROS topic/service/node graph |
| `ros-agent://workflows` | Saved compose workflows |
| `ros-agent://prompts-catalog` | All available prompts |
| `ros-agent://soul` | Agent behavioral identity |
| `ros-agent://topic/{path}` | Live read from any ROS topic (template) |

---

## Skills

Plugin-based. Drop a Python file, get new MCP tools.

```
Built-in                            → ships with ros-agent
ROBORUN_SKILL_PACKAGES=pkg1,pkg2    → pip packages
ROBORUN_SKILL_PATHS=/path/to/dir    → filesystem directories
.roborun/skills.yaml                → project-level config
```

### Built-in Skills

| Skill | Tools | What it does |
|-------|:-----:|-------------|
| **compose** | 5 | Chain tools into reusable workflows — `run_sequence`, `save_workflow`, `run_workflow` |
| **inspect** | 3 | `robot_brief` (one-call overview), `watch_topic` (conditional monitor), `diff_state` (graph changes) |
| **follow_me** | 1 | Visual person-following with P-control on camera feed |
| **patrol** | 5 | Autonomous waypoint patrol loop with configurable dwell times |
| **scan_detect** | 2 | Rotate-and-detect object search using YOLO + CLIP fallback |

### Write Your Own

```python
SKILL_TOOLS = [
    {"name": "my_tool", "description": "Does a thing",
     "inputSchema": {"type": "object", "properties": {"x": {"type": "string"}}}}
]

def handle(name: str, args: dict) -> str:
    if name == "my_tool":
        return f"Got: {args.get('x')}"
```

Point `ROBORUN_SKILL_PATHS` at the directory. Done.

---

## Connect a Robot

ros-agent connects to any robot running `rosbridge_server` (ROS 1 or ROS 2) over WebSocket. No ROS installation needed on your machine.

```bash
# On a ROS 2 robot
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# On a ROS 1 robot
roslaunch rosbridge_server rosbridge_websocket_launch.launch

# From ros-agent — set IP in the UI, or:
curl -X POST http://localhost:8765/api/ros/connect \
  -d '{"host":"192.168.1.100"}'
```

Direct DDS transport also available for zero-latency local use:

```bash
pip install ros-agent[ros]   # adds CycloneDDS + ros_tap
```

---

## Configuration

| Variable | Default | Description |
|----------|---------|-------------|
| `ROBORUN_PORT` | `8765` | Server port |
| `ROBOT_IP` | — | Robot IP (can also set in UI) |
| `ANTHROPIC_API_KEY` | — | Enables Claude agent |
| `GEMINI_API_KEY` | — | Enables Gemini agent |
| `ROBORUN_SKILL_PACKAGES` | — | Additional skill packages |
| `ROBORUN_SKILL_PATHS` | — | Additional skill directories |
| `ROBORUN_MAX_LINEAR_VEL` | `1.0` | Safety velocity limit (m/s) |
| `ROBORUN_MAX_ANGULAR_VEL` | `1.5` | Safety angular limit (rad/s) |

---

## Contributing

```bash
git clone https://github.com/publu/RoboRun.git
cd RoboRun
pip install -e ".[all]"
python -m roborun.server  # internal module name
```

---

## License

MIT — see [LICENSE](LICENSE).

Built by [Hashing Systems](https://hashingsystems.com).
