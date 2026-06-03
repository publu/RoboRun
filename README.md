# RoboRun

**Zero-terminal visual teleop, dataset collection, and one-click dimOS blueprint deployment.**

RoboRun is a visual dashboard for robot control and computer vision. Use it with just a webcam to run YOLO, CLIP, and JEPA models in real time -- or connect to [dimOS](https://github.com/dimensionalOS/dimos) for full robot control with navigation, exploration, smart follow, dog mode, and more.

No CLI required. Everything runs from the browser.

[![Demo](assets/demo-thumb.jpg)](https://github.com/publu/RoboRun/raw/main/assets/demo.mp4)

---

## Changelog

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

### 1. Install

```bash
git clone https://github.com/publu/RoboRun.git
cd RoboRun
python3 -m venv .venv && source .venv/bin/activate
pip install -e .
```

### 2. Run

```bash
python -m roborun.server
```

Open **http://127.0.0.1:8765** in your browser. Your webcam starts automatically.

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

## Architecture

```
Browser (RoboRun UI)
    |
    +-- /api/webcam/*      -> WebcamPipeline (YOLO + CLIP + JEPA)
    +-- /api/dataset/*     -> DatasetCollector (episode recording)
    +-- /api/mcp/call      -> Daneel MCP (:9990) -> dimOS -> Robot
    +-- /api/agent/chat    -> Claude CLI (stream-json) -> MCP tools
    +-- /api/fleet/*       -> Fleet + Blueprint management
    +-- /api/tasks/*       -> Task scheduler
    +-- /api/camera/stream -> MJPEG (webcam or robot camera)
```

**dimOS is optional.** The webcam pipeline, model inference, and recording all work standalone. When dimOS is available, RoboRun adds full robot control on top.

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

## Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `ROBORUN_PORT` | `8765` | Server port |
| `ROBOT_IP` | -- | Robot IP (can also set in UI) |

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
