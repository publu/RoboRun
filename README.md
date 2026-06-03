# RoboRun

**Zero-terminal visual teleop, dataset collection, and one-click dimOS blueprint deployment.**

RoboRun is a visual dashboard for robot control and computer vision. Use it with just a webcam to test YOLO, CLIP, JEPA, and Cosmos models in real time — or connect to [dimOS](https://github.com/dimensionalOS/dimos) for full robot control with navigation, exploration, smart follow, dog mode, and more.

No CLI required. Everything runs from the browser.

---

## Features

### Webcam + Vision Models
- **YOLO** — real-time object detection and tracking on your webcam
- **CLIP** — zero-shot text-image search ("find the red mug", "person in blue jacket")
- **JEPA** — self-supervised visual representations (V-JEPA encoder)
- **Cosmos** — NVIDIA's discrete visual tokenizer for world model training

Toggle models on and off from the dashboard. See detections overlaid on your live feed instantly.

### Dataset Collection
- Record episodes from webcam or robot camera
- Frames saved with YOLO detections and CLIP annotations
- Browse and manage datasets from the UI
- Export-ready format for training pipelines

### dimOS Robot Control
- **One-click blueprint deployment** — Full Dashboard, Standard Go2, Agentic (Claude/Ollama), Security Patrol, Spatial Memory, and more
- **Visual teleop** — WASD movement, D-pad, step size control
- **Smart skills** — explore, navigate, follow person/object, find, dog mode, patrol, speak
- **Direct MCP calls** — zero-latency skill dispatch (no AI round-trip)
- **AI agent chat** — Claude-powered operator with tool use, thinking, streaming
- **Fleet management** — add robots, assign blueprints, deploy at scale
- **Task scheduler** — one-off and recurring tasks with GPS, map, and AI query actions
- **Live map** — embedded Command Center with Socket.IO pose tracking
- **System monitoring** — resource stats, event log, dimOS status

---

## Quick Start

### 1. Install

```bash
git clone https://github.com/publu/RoboRun.git
cd RoboRun
pip install -e .
```

### 2. Run

```bash
python -m roborun.server
```

Open **http://127.0.0.1:8765** in your browser.

### 3. Start your webcam

Click **Vision** in the sidebar, select your camera and models, hit **Start Webcam**. The live feed with detections appears in the Control tab.

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
    │
    ├── /api/webcam/*      → WebcamPipeline (YOLO + CLIP + JEPA + Cosmos)
    ├── /api/dataset/*     → DatasetCollector (episode recording)
    ├── /api/mcp/call      → Daneel MCP (:9990) → dimOS → Robot
    ├── /api/agent/chat    → Claude CLI (stream-json) → MCP tools
    ├── /api/fleet/*       → Fleet + Blueprint management
    ├── /api/tasks/*       → Task scheduler
    └── /api/camera/stream → MJPEG (webcam or robot camera)
```

**dimOS is optional.** The webcam pipeline, model inference, and dataset collection all work standalone. When dimOS is available, RoboRun adds full robot control on top.

You can also point RoboRun to your own dimOS checkout by setting the path in System > Profile.

---

## Models

| Model | Package | What it does | Install |
|-------|---------|-------------|---------|
| YOLO | `ultralytics` | Object detection + tracking | Included |
| CLIP | `open-clip-torch` | Text-image matching, zero-shot search | Included |
| JEPA | `timm` | Self-supervised visual features | `pip install timm` |
| Cosmos | `cosmos-tokenizer` | Discrete video tokenization | `pip install cosmos-tokenizer` |

JEPA and Cosmos are optional. Install them separately if you want to use them:

```bash
pip install -e ".[all]"  # installs everything
```

---

## Configuration

All settings are in the UI under **System > Profile**:

| Setting | Description |
|---------|-------------|
| Device name | Display name for your station |
| Robot IP | Your Go2's IP (e.g., `192.168.123.18`) |
| dimOS path | Path to your dimOS checkout (optional) |
| Blueprint | Which dimOS blueprint to launch |
| Camera index | Which webcam to use (0 = default) |

Settings persist in `.roborun/profile.json`.

---

## Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `ROBORUN_PORT` | `8765` | Server port |
| `ROBOT_IP` | — | Robot IP (can also set in UI) |

---

## Project Structure

```
RoboRun/
├── roborun/
│   ├── server.py      # HTTP server + all API routes
│   ├── webcam.py       # Webcam capture + model pipeline
│   ├── models.py       # YOLO, CLIP, JEPA, Cosmos wrappers
│   ├── dataset.py      # Episode recording + management
│   └── agent.py        # Claude agent (optional)
├── web/
│   ├── index.html      # Dashboard UI
│   ├── app.js          # Frontend logic
│   └── styles.css      # Dark HUD theme
├── scripts/
│   └── start.sh        # Launch script
├── pyproject.toml      # Package config
└── README.md
```

---

## License

MIT
