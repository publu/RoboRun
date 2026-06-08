# RoboRun

**The agentic robot OS with real-time vision AI, spatial memory, and native ROS 2 transport.**

```
npx roborun
```

Open your browser. Your webcam now has YOLO, CLIP, and JEPA running in real time. Add a robot IP and you have full fleet control, spatial memory, a Claude agent, a MuJoCo physics simulator, and native ROS 2 topic pub/sub over rosbridge — all from a single browser tab.

**Works with any AI client. Claude Desktop, Claude Code, Cursor — add one line to your MCP config:**

```json
{ "mcpServers": { "roborun": { "type": "http", "url": "http://localhost:8765/mcp" } } }
```

Your AI gets: live camera feed, YOLO detections, move commands, robot skills, spatial memory search, ROS 2 topic access.

**Works with any robot. No ROS installation required.**

- Webcam-only: pip install, open browser, done
- dimOS robots (Unitree Go2/G1): one-click blueprint deploy
- Any ROS 2 robot with rosbridge_server: `/api/ros/*` endpoints match the agenticROS tool surface exactly
- Any HTTP camera or video stream: plug in the URL

[![Demo](assets/demo-thumb.jpg)](https://github.com/publu/RoboRun/raw/main/assets/demo.mp4)

[![Demo](assets/demo-thumb.jpg)](https://github.com/publu/RoboRun/raw/main/assets/demo.mp4)

---

## Changelog

### v0.3.0 -- MuJoCo simulator, spatial memory, walking robots

- **MuJoCo simulator in the browser** -- click Simulate on the Control tab, pick a robot (Go1 or G1), and launch a full physics sim. Same camera feed, same WASD controls. 3rd-person camera tracks the robot.
- **Trained locomotion policies** -- Go1 and G1 walk for real using ONNX neural network controllers from dimOS. WASD sends velocity commands to the policy which outputs joint torques. Actual walking gait, not position hacking.
- **Spatial memory store** -- CLIP-searchable, geo-indexed, multi-robot memory. Store webcam frames with embeddings, YOLO detections, and x/y/z coordinates. Search by text query (CLIP cosine similarity), spatial radius, or YOLO label. Optional S3 backend for thumbnails, search index always local.
- **Simulator integrated into Control tab** -- no separate tab. Source picker lets you switch between Webcam and Simulate. Sidebar status reflects sim state. Robot controls auto-expand when sim is active.

### v0.2.0 -- Cosmos 3, JEPA heatmaps, UX overhaul

- **Cosmos 3 Nano world model** -- integrated via [cosmos-mac](https://github.com/publu/cosmos-mac) (MLX 4-bit, Apple Silicon native). Available as a Python API for world simulation, synthetic data, and action prediction. Not in the live webcam loop (generation takes ~20s), but wired in the codebase for downstream use.
- **JEPA attention heatmaps** -- toggle JEPA in the model bar to see a real-time ViT activation overlay on your webcam feed. Shows what the self-supervised encoder focuses on.
- **CLIP zero-shot search** -- type a query ("red mug", "person in blue jacket") and matching YOLO detections get highlighted in real-time.
- **Webcam auto-start** -- the camera starts automatically when you open the dashboard. No more hunting for a start button.
- **Robot controls collapsed by default** -- when no robot is connected, the 24 robot action buttons are tucked away. They auto-expand when dimOS comes online.
- **Collapsible agent panel** -- click the toggle to hide the chat panel and get a bigger camera view.
- **Model failure toasts** -- if a model isn't installed, you get a visible error instead of silent failure.
- **Recordings tab** -- renamed from "Dataset" because calling video clips "datasets" was confusing.
- **Resilient webcam pipeline** -- if a model crashes, it gets disabled and the stream keeps running instead of dying.

### v0.1.0 -- Initial release

- Zero-terminal visual teleop with WASD movement
- YOLO real-time object detection and tracking
- CLIP text-image search
- Dataset/episode recording from webcam or robot camera
- dimOS one-click blueprint deployment (Full Dashboard, Standard Go2, Agentic, Security Patrol, Spatial Memory, etc.)
- AI agent chat with Claude tool use and streaming
- Fleet management, task scheduler, live map
- Direct MCP skill dispatch (no AI round-trip)

---

## vs agenticROS

[agenticROS](https://github.com/agenticros/agenticros) bridges ROS 2 robots to AI agents. RoboRun does that too, plus everything else:

| Capability | RoboRun | agenticROS |
|---|:---:|:---:|
| Native ROS 2 transport (rosbridge) | Yes | Yes |
| Raw topic pub/sub/service/action | Yes (`/api/ros/*`) | Yes (MCP tools) |
| Works without ROS (webcam-only) | **Yes** | No |
| Real-time YOLO object detection | **Yes** | No |
| CLIP zero-shot search | **Yes** | No |
| JEPA attention heatmap | **Yes** | No |
| Spatial memory (geo-indexed, searchable) | **Yes** | No |
| Physics simulation (MuJoCo, in browser) | **Yes** | Gazebo (Linux only) |
| Dataset/episode recording | **Yes** | No |
| Cosmos 3 world model | **Yes** | No |
| ZK-verified observations (EZKL) | **Yes** (`pip install roborun[zk]`) | No |
| Multi-agent (Claude + Gemini) | **Yes** | Yes |
| Single-command install | `npx roborun` | `npx agenticros` |
| Fleet management UI | **Yes** | No |
| EU AI Act compliance path | **Yes** (ZK proofs) | No |

RoboRun's `/api/ros/*` endpoints use the same tool names and schemas as agenticROS's MCP tools — any agenticROS skill or prompt works unmodified.

---

## Features

### Webcam + Vision Models

| Model | What it does | Live overlay | Install |
|-------|-------------|:---:|---------|
| **YOLO** | Real-time object detection + tracking | Bounding boxes + IDs | `ultralytics` (included) |
| **CLIP** | Zero-shot text-image search | Highlighted matches | `open-clip-torch` (included) |
| **JEPA** | Self-supervised visual representations | Attention heatmap | `timm` (included) |
| **Cosmos 3 Nano** | 16B world foundation model (MLX 4-bit) | API only | [cosmos-mac](https://github.com/publu/cosmos-mac) |

Toggle YOLO, CLIP, and JEPA from the model bar on the Control tab. They run in real-time on your webcam feed.

Cosmos 3 is available as `CosmosWorldModel` in `roborun/models.py` for programmatic use -- world simulation, synthetic data generation, action-conditioned prediction. It runs on Apple Silicon via MLX 4-bit quantization (~20s per image, ~11GB peak memory).

### Recordings

Record video clips from your webcam or robot camera with model annotations baked in. Browse and manage recordings from the Recordings tab.

### dimOS Robot Control

- **One-click blueprint deployment** -- Full Dashboard, Standard Go2, Agentic (Claude/Ollama), Security Patrol, Spatial Memory, and more
- **Visual teleop** -- WASD movement, D-pad, step size control
- **Smart skills** -- explore, navigate, follow person/object, find, dog mode, patrol, speak
- **Direct MCP calls** -- zero-latency skill dispatch (no AI round-trip)
- **AI agent chat** -- Claude-powered operator with tool use, thinking, streaming
- **Fleet management** -- add robots, assign blueprints, deploy at scale
- **Task scheduler** -- one-off and recurring tasks with GPS, map, and AI query actions
- **Live map** -- embedded Command Center with Socket.IO pose tracking
- **System monitoring** -- resource stats, event log, dimOS status

---

## Quick Start

### Option A — one command (Node 18+ required)

```bash
npx roborun
```

Checks Python 3.10+, installs `roborun` via pip, starts the server, opens the browser.

### Option B — pip install

```bash
pip install roborun
roborun
```

Open **http://127.0.0.1:8765**. Your webcam starts automatically.

### Option C — from source

```bash
git clone https://github.com/publu/RoboRun.git
cd RoboRun
python3 -m venv .venv && source .venv/bin/activate
pip install -e .
python -m roborun.server
```

### 3. (Optional) Cosmos 3 setup

To use the Cosmos 3 world model:

```bash
git clone https://github.com/publu/cosmos-mac.git
cd cosmos-mac
# Follow cosmos-mac README to download the MLX 4-bit weights
```

Then in Python:

```python
from roborun.models import CosmosWorldModel

cosmos = CosmosWorldModel(cosmos_dir="/path/to/cosmos-mac")
frame = cosmos.generate(prompt="A robot picks up a red cube", steps=12, resolution=256)
```

### 4. (Optional) Connect dimOS

If you have dimOS installed and a Unitree Go2 on the network:

```bash
# Set your robot IP in System > Profile, then click Start Robot
# Or use replay mode (no robot needed):
python -m roborun.server  # then click "Replay Bot" in System tab
```

---

## ROS 2 Native Transport

RoboRun connects to any robot running `rosbridge_server` via WebSocket. The `/api/ros/*` endpoints match the agenticROS MCP tool surface — any agenticROS skill or prompt works with RoboRun unmodified.

```bash
# On the robot (any ROS 2 distro)
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# In your profile settings, set Robot IP to the robot's IP
# Then use the ROS tab or call the API directly:
curl -X POST http://localhost:8765/api/ros/topics
curl -X POST http://localhost:8765/api/ros/publish \
  -d '{"topic":"/cmd_vel","type":"geometry_msgs/Twist","message":{"linear":{"x":0.5}}}'
curl -X POST http://localhost:8765/api/ros/subscribe-once \
  -d '{"topic":"/scan","timeout":5000}'
```

| Endpoint | Description |
|---|---|
| `GET  /api/ros/topics` | List all ROS 2 topics |
| `GET  /api/ros/status` | Check rosbridge connection |
| `POST /api/ros/connect` | Connect to rosbridge host |
| `POST /api/ros/publish` | Publish to any topic |
| `POST /api/ros/subscribe-once` | Read one message from a topic |
| `POST /api/ros/service` | Call a ROS 2 service |
| `POST /api/ros/action` | Send an action goal |
| `POST /api/ros/param/get` | Read a node parameter |
| `POST /api/ros/param/set` | Set a node parameter |
| `POST /api/ros/camera` | Grab a camera frame (CompressedImage or raw) |
| `POST /api/ros/depth` | Sample center depth in meters |
| `POST /api/ros/move` | Publish Twist to /cmd_vel |

**dimOS is optional.** RoboRun works with dimOS robots (full MCP skill set) or any ROS 2 robot (raw rosbridge transport) or just a webcam (no robot required).

## Architecture

```
Browser (RoboRun UI)
    |
    +-- /api/webcam/*      -> WebcamPipeline (YOLO + CLIP + JEPA)
    +-- /api/dataset/*     -> DatasetCollector (episode recording)
    +-- /api/ros/*         -> RosbridgeClient (any ROS 2 robot)
    +-- /api/mcp/call      -> Daneel MCP (:9990) -> dimOS -> Robot
    +-- /api/agent/chat    -> Claude CLI (stream-json) -> MCP tools
    +-- /api/fleet/*       -> Fleet + Blueprint management
    +-- /api/tasks/*       -> Task scheduler
    +-- /api/camera/stream -> MJPEG (webcam or robot camera)
    +-- /api/memory/*      -> SpatialMemoryStore (CLIP search + S3)
```

For a fleet-scale version of this architecture, see [docs/realtime-data-plane.md](docs/realtime-data-plane.md). The short version: keep local mode lightweight, use MQTT or NATS for robot events and command acknowledgements, use Postgres/Supabase for operator and fleet state, use Timescale/Influx/QuestDB for telemetry history, and keep high-bandwidth robotics logs in Foxglove/MCAP plus object storage.

---

## Models

| Model | Package | What it does | Install |
|-------|---------|-------------|---------|
| YOLO | `ultralytics` | Object detection + tracking | Included |
| CLIP | `open-clip-torch` | Text-image matching, zero-shot search | Included |
| JEPA | `timm` | Self-supervised visual features (ViT) | Included |
| Cosmos 3 Nano | `mlx` + `diffusers` | 16B world model (text/image/video/action) | [cosmos-mac](https://github.com/publu/cosmos-mac) |

---

## Configuration

All settings are in the UI under **System > Profile**:

| Setting | Description |
|---------|-------------|
| Device name | Display name for your station |
| Robot IP | Your Go2's IP (e.g., `192.168.123.18`) |
| dimOS path | Path to your dimOS checkout (optional) |
| Blueprint | Which dimOS blueprint to launch |

Settings persist in `.roborun/profile.json`.

---

## ZK-Verified Observations

RoboRun can generate cryptographic proofs that CLIP embeddings in the spatial memory were correctly computed from the original frames. This enables post-hoc verification of what the robot actually saw — for compliance, insurance, and trust.

```bash
pip install "roborun[zk]"  # installs ezkl + onnx

# One-time circuit setup (~2-5 min)
curl -X POST http://localhost:8765/api/zk/setup

# Check status
curl http://localhost:8765/api/zk/status

# Generate proof for a memory shard
curl -X POST http://localhost:8765/api/zk/prove \
  -d '{"shard_id": "abc123"}'

# Verify a proof
curl http://localhost:8765/api/zk/verify/abc123
# → {"verified": true, "model": "CLIP ViT-B/32", "frame_count": 128}
```

The proof file travels with the data: anyone with `shard.proof.bin` + the original frames can verify independently, without trusting your server.

Proof generation: ~30-120s per shard on 1 GPU (EZKL CLIP ViT-B/32). Verification: ~50ms. Storage: ~2MB per shard proof.

EU AI Act high-risk AI systems (autonomous robots in workplaces) require audit-grade logs by August 2026. ZK proofs satisfy this without exposing proprietary model weights.

---

## Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `ROBORUN_PORT` | `8765` | Server port |
| `ROBOT_IP` | -- | Robot IP (can also set in UI) |
| `GEMINI_API_KEY` | -- | Enables Gemini agent (`/api/agent/gemini`) |
| `ROBORUN_S3_BUCKET` | -- | S3/R2 bucket for spatial memory thumbnails |
| `ROBORUN_S3_ENDPOINT` | -- | Custom S3 endpoint (Cloudflare R2, MinIO, etc.) |

---

## Project Structure

```
RoboRun/
+-- roborun/
|   +-- server.py      # HTTP server + all API routes
|   +-- webcam.py       # Webcam capture + model pipeline
|   +-- models.py       # YOLO, CLIP, JEPA, Cosmos wrappers
|   +-- dataset.py      # Episode recording + management
|   +-- agent.py        # Claude agent (optional)
+-- web/
|   +-- index.html      # Dashboard UI
|   +-- app.js          # Frontend logic
|   +-- styles.css      # Dark HUD theme
+-- scripts/
|   +-- start.sh        # Launch script
+-- pyproject.toml      # Package config
+-- README.md
```

---

## License

MIT
