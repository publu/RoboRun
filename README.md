# RoboRun

**Agentic robot OS — skills, fleet, vision AI, and MCP for any ROS 2 robot.**

```
pip install roborun
roborun
```

Open **http://127.0.0.1:8765**. Your webcam starts with YOLO, CLIP, and JEPA running in real time. Add a robot IP and you get full fleet control, spatial memory, a Claude agent, MuJoCo simulation, and native ROS 2 transport — all from one browser tab.

**Works with any MCP client.** Claude Desktop, Claude Code, Cursor — add one line:

```json
{ "mcpServers": { "roborun": { "type": "http", "url": "http://localhost:8765/mcp" } } }
```

Or use the stdio transport directly:

```json
{ "mcpServers": { "roborun": { "command": "roborun-mcp" } } }
```

Your AI gets: 49 tools, 8 guided prompts, 6 resources, live topic streaming, robot skills.

---

## Quick Start

### Option A — pip install

```bash
pip install roborun
roborun
```

### Option B — from source

```bash
git clone https://github.com/publu/RoboRun.git
cd roborun
python3 -m venv .venv && source .venv/bin/activate
pip install -e .
python -m roborun.server
```

### Option C — npx (Node 18+)

```bash
npx roborun
```

Open **http://127.0.0.1:8765**. Webcam starts automatically.

---

## MCP Server

RoboRun exposes a full MCP server with two transports:

| Transport | Endpoint | Use case |
|-----------|----------|----------|
| HTTP+SSE | `http://localhost:8765/mcp` | Claude Desktop, Cursor, web clients |
| stdio | `roborun-mcp` | Claude Code, CLI-based clients |

### 49 Tools

30 built-in ROS tools (topic pub/sub, services, actions, params, camera, depth, velocity control, introspection) plus 19 skill tools from 5 built-in skills.

### 8 Prompts

| Prompt | Description |
|--------|-------------|
| `explore-robot` | Guided robot discovery workflow |
| `safety-check` | Pre-operation safety verification |
| `environment-scan` | Full environment survey |
| `teach-waypoints` | Interactive waypoint teaching |
| `debug-topic` | Diagnose a misbehaving topic |
| `quick-start` | First-time user onboarding |
| `fleet-sweep` | Multi-robot status check |
| `build-workflow` | Create a reusable workflow |

### 6 Resources + Templates

| Resource | Description |
|----------|-------------|
| `roborun://server-info` | Server version, uptime, capabilities |
| `roborun://skills` | Loaded skills and their tools |
| `roborun://ros-graph` | Live ROS topic/service/node graph |
| `roborun://workflows` | Saved compose workflows |
| `roborun://prompts-catalog` | All available prompts |
| `roborun://soul` | Agent behavioral identity (SOUL.md) |
| `roborun://topic/{path}` | Live read from any ROS topic (template) |

---

## Skills

RoboRun uses a plugin-based skills system. Skills are loaded from 4 sources:

1. **Built-in** — shipped with RoboRun
2. **`ROBORUN_SKILL_PACKAGES`** — comma-separated pip packages
3. **`ROBORUN_SKILL_PATHS`** — comma-separated filesystem paths
4. **`.roborun/skills.yaml`** — project-level skill config

### Built-in Skills

| Skill | Tools | Description |
|-------|-------|-------------|
| **compose** | `run_sequence`, `save_workflow`, `run_workflow`, `list_workflows`, `delete_workflow` | Chain tools into reusable workflows |
| **inspect** | `robot_brief`, `watch_topic`, `diff_state` | High-level robot introspection |
| **follow_me** | `follow_target_person` | Visual person-following with P-control |
| **patrol** | `start_patrol`, `stop_patrol`, `add_patrol_waypoint` | Autonomous waypoint patrol |
| **scan_detect** | `scan_and_detect`, `find_object` | Rotate-and-detect object search |

### Writing a Skill

Create a Python file with `SKILL_TOOLS` (list of MCP tool dicts) and `handle(name, args)`:

```python
SKILL_TOOLS = [
    {"name": "my_tool", "description": "Does a thing",
     "inputSchema": {"type": "object", "properties": {"x": {"type": "string"}}}}
]

def handle(name: str, args: dict) -> str:
    if name == "my_tool":
        return f"Got: {args.get('x')}"
```

Drop it in a directory and set `ROBORUN_SKILL_PATHS=/path/to/skills`.

---

## ROS 2 Transport

RoboRun connects to any robot running `rosbridge_server` via WebSocket. No ROS installation needed on the RoboRun host.

```bash
# On the robot
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Set Robot IP in the UI, or:
curl -X POST http://localhost:8765/api/ros/connect -d '{"host":"192.168.1.100"}'

# Use any tool
curl -X POST http://localhost:8765/api/ros/topics
curl -X POST http://localhost:8765/api/ros/publish \
  -d '{"topic":"/cmd_vel","type":"geometry_msgs/Twist","message":{"linear":{"x":0.5}}}'
```

Also supports direct DDS via `ros_tap` + CycloneDDS (`pip install roborun[ros]`).

---

## Vision Models

| Model | What it does | Live overlay | Install |
|-------|-------------|:---:|---------|
| **YOLO** | Object detection + tracking | Bounding boxes + IDs | `pip install roborun[vision]` |
| **CLIP** | Zero-shot text-image search | Highlighted matches | `pip install roborun[vision]` |
| **JEPA** | Self-supervised visual features | Attention heatmap | `pip install roborun[jepa]` |
| **Cosmos 3** | 16B world model (MLX 4-bit) | API only | [cosmos-mac](https://github.com/publu/cosmos-mac) |

Toggle models from the model bar in the UI. They run in real-time on your webcam or robot camera.

---

## Agent

Built-in Claude agent with tool use, streaming, and vision. Also supports Gemini.

The agent gets dynamic ROS context injection — live topics, services, and nodes are summarized in its system prompt. Safety velocity clamping prevents runaway commands. Persistent cross-agent memory stores facts across sessions.

Agent behavioral identity is defined in `.roborun/SOUL.md` — safety rules, interaction style, tool preferences.

---

## Architecture

```
Browser (RoboRun UI)
    |
    +-- /api/webcam/*      -> WebcamPipeline (YOLO + CLIP + JEPA)
    +-- /api/dataset/*     -> DatasetCollector (episode recording)
    +-- /api/ros/*         -> RosbridgeClient (any ROS 2 robot)
    +-- /api/agent/chat    -> Claude/Gemini agent with MCP tools
    +-- /api/fleet/*       -> Fleet + Blueprint management
    +-- /api/tasks/*       -> Task scheduler
    +-- /api/memory/*      -> SpatialMemoryStore (CLIP search)
    +-- /api/sim/*         -> MuJoCo physics simulation
    +-- /mcp               -> MCP HTTP+SSE transport
    +-- roborun-mcp        -> MCP stdio transport
```

### Project Structure

```
roborun/
+-- server.py          # Thin HTTP shell, route dispatch
+-- ros_mcp.py         # 30 built-in ROS MCP tools
+-- mcp_stdio.py       # MCP stdio transport (prompts, resources, logging)
+-- agent.py           # Claude + Gemini agent implementations
+-- rosbridge.py       # WebSocket ROS 2 transport
+-- simulator.py       # MuJoCo headless simulation
+-- webcam.py          # Webcam capture + model pipeline
+-- spatial_memory.py  # CLIP-indexed geo-searchable memory
+-- skills/            # Plugin skill modules
+-- routes/            # HTTP route handlers (12 modules)
web/
+-- index.html         # Dashboard UI
+-- app.js             # Frontend logic
+-- styles.css         # Dark HUD theme
```

---

## Configuration

Settings are in the UI under **System > Profile** and persist in `.roborun/profile.json`.

| Setting | Description |
|---------|-------------|
| Device name | Display name for your station |
| Robot IP | Robot's IP address (e.g., `192.168.123.18`) |
| Blueprint | Robot blueprint to use |

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `ROBORUN_PORT` | `8765` | Server port |
| `ROBOT_IP` | -- | Robot IP (can also set in UI) |
| `ANTHROPIC_API_KEY` | -- | Enables Claude agent |
| `GEMINI_API_KEY` | -- | Enables Gemini agent |
| `ROBORUN_SKILL_PACKAGES` | -- | Additional skill packages |
| `ROBORUN_SKILL_PATHS` | -- | Additional skill directories |
| `ROBORUN_MAX_LINEAR_VEL` | `1.0` | Safety velocity limit (m/s) |
| `ROBORUN_MAX_ANGULAR_VEL` | `1.5` | Safety angular limit (rad/s) |
| `ROBORUN_S3_BUCKET` | -- | S3/R2 bucket for memory thumbnails |

### Optional Dependencies

```bash
pip install roborun[vision]   # YOLO + CLIP + OpenCV
pip install roborun[ros]      # Direct DDS (ros_tap + CycloneDDS)
pip install roborun[zk]       # ZK proofs (EZKL + ONNX)
pip install roborun[gemini]   # Gemini agent
pip install roborun[all]      # Everything
```

---

## ZK-Verified Observations

Generate cryptographic proofs that CLIP embeddings were correctly computed from original frames.

```bash
pip install "roborun[zk]"
curl -X POST http://localhost:8765/api/zk/setup    # One-time circuit setup
curl -X POST http://localhost:8765/api/zk/prove -d '{"shard_id": "abc123"}'
curl http://localhost:8765/api/zk/verify/abc123
```

---

## Changelog

### v0.8.0 — Skills, MCP prompts/resources, codebase overhaul

- **Skills plugin system** — 4 loading paths (built-in, packages, paths, config), 5 built-in skills (compose, inspect, follow_me, patrol, scan_detect), skill template for easy authoring
- **Compose skill** — chain any tools into reusable workflows with `run_sequence`, persist and replay with `save_workflow` / `run_workflow`
- **Inspect skill** — `robot_brief` (one-call robot overview), `watch_topic` (conditional monitoring), `diff_state` (detect ROS graph changes)
- **8 MCP prompts** — guided workflows: explore-robot, safety-check, environment-scan, teach-waypoints, debug-topic, quick-start, fleet-sweep, build-workflow
- **6 MCP resources + 1 template** — server-info, skills, ros-graph, workflows, prompts-catalog, soul. Resource template: `roborun://topic/{path}` for live topic reads
- **MCP logging** — real-time `notifications/message` on every tool call
- **49 total tools** — 30 built-in ROS tools + 19 skill tools
- **Route decomposition** — server.py shrunk from 2300 to 200 lines, 12 focused route modules
- **Generic codebase** — removed all hardcoded dimOS/Go2 references, works with any ROS 2 robot
- **Configurable stack integration** — `stackCommand` in profile settings, defaults to `dimos` for backwards compatibility
- **Safety velocity clamping** — configurable via `ROBORUN_MAX_LINEAR_VEL` / `ROBORUN_MAX_ANGULAR_VEL`
- **Dynamic ROS context** — agent system prompt auto-injects live topics, nodes, transports
- **Persistent cross-agent memory** — facts persist across sessions in `.roborun/agent_memory.json`
- **SOUL.md** — agent behavioral identity (safety rules, interaction style)
- **Thin dependencies** — core requires only `websocket-client` + `websockets`

### v0.7.0 — Unified ROS MCP, DDS + rosbridge

- **30 ROS MCP tools** — full introspection (topic/node/service/action details), pub/sub, services, actions, params, camera, depth, velocity
- **Dual transport** — rosbridge WebSocket + direct DDS via `ros_tap` + CycloneDDS
- **MCP stdio transport** — `roborun-mcp` CLI for Claude Code, any stdio MCP client

### v0.6.0 — 3D scene builder, ROS telemetry

- **3D scene builder** — depth-based point cloud reconstruction from webcam
- **ROS telemetry bridge** — auto-subscribes to battery, IMU, odom, joint states
- **WebSocket telemetry** — real-time charts at ws://127.0.0.1:8766

### v0.5.0 — 3D spatial perception, drone support, telemetry dashboard

- **Telemetry dashboard** — WebSocket-powered charts for battery, altitude, speed, orientation, joints
- **3D trajectory** — Three.js path visualization with orbit controls
- **Point cloud viewer** — colored depth/LiDAR point cloud
- **Drone support** — MuJoCo quadrotor with PID controller, 6 new tools
- **Robot type system** — UI adapts based on robot type (drone, quadruped, humanoid, webcam)

### v0.4.0 — MCP server, native ROS 2, fast agent

- **MCP HTTP server at `/mcp`** — any AI client connects and gets robot tools
- **Native ROS 2 transport** — rosbridge WebSocket to any robot
- **Fast SDK agent** — direct Anthropic SDK streaming with sensor pre-injection
- **ZK proof layer** — EZKL proofs for CLIP embeddings

### v0.3.0 — MuJoCo simulator, spatial memory, walking robots

- **MuJoCo sim** — full physics, trained ONNX locomotion policies
- **Spatial memory** — CLIP-searchable, geo-indexed, multi-robot
- **Simulator in browser** — same camera feed, same WASD controls

### v0.2.0 — Cosmos 3, JEPA heatmaps, UX overhaul

- **Cosmos 3 Nano** — 16B world model via MLX 4-bit
- **JEPA attention heatmaps** — ViT activation overlay
- **CLIP zero-shot search** — real-time detection highlighting

### v0.1.0 — Initial release

- WASD teleop, YOLO detection, CLIP search, dataset recording
- AI agent chat with Claude, fleet management, task scheduler

---

## License

MIT
