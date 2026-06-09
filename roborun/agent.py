"""Robot agents for RoboRun — Claude (stream-json/MCP) and Gemini (function calling).

Claude agent: persistent subprocess using the claude CLI with MCP servers for
dimOS robot control and the RoboRun workbench.

Fast agent: direct Anthropic SDK with multimodal sensor injection, safety
velocity limits, dynamic ROS context, and persistent memory.

Gemini agent: stateless function-calling loop using google-generativeai. Exposes
the same robot tool surface as agenticROS's MCP tools, no MCP required.
"""
from __future__ import annotations

import json
import logging
import os
import subprocess
import threading
import time
import urllib.request
from pathlib import Path
from typing import Iterator

log = logging.getLogger(__name__)

ROOT = Path(__file__).resolve().parent.parent
SESSION_FILE = ROOT / ".roborun" / "agent_session.txt"
MEMORY_FILE = ROOT / ".roborun" / "agent_memory.json"
SOUL_FILE = ROOT / ".roborun" / "SOUL.md"

# ── Safety limits ──────────────────────────────────────────────────────────────

MAX_LINEAR_VEL = float(os.environ.get("ROBORUN_MAX_LINEAR_VEL", "1.0"))
MAX_ANGULAR_VEL = float(os.environ.get("ROBORUN_MAX_ANGULAR_VEL", "1.5"))


def _clamp_velocity(linear_x: float = 0.0, linear_y: float = 0.0,
                    angular_z: float = 0.0) -> tuple[float, float, float]:
    clamped = False
    lx = max(-MAX_LINEAR_VEL, min(MAX_LINEAR_VEL, linear_x))
    ly = max(-MAX_LINEAR_VEL, min(MAX_LINEAR_VEL, linear_y))
    az = max(-MAX_ANGULAR_VEL, min(MAX_ANGULAR_VEL, angular_z))
    if lx != linear_x or ly != linear_y or az != angular_z:
        clamped = True
        log.warning("Velocity clamped: (%.2f,%.2f,%.2f) -> (%.2f,%.2f,%.2f)",
                    linear_x, linear_y, angular_z, lx, ly, az)
    return lx, ly, az


# ── Persistent memory ──────────────────────────────────────────────────────────

_memory_lock = threading.Lock()


def _load_memory() -> list[dict]:
    with _memory_lock:
        try:
            return json.loads(MEMORY_FILE.read_text()) if MEMORY_FILE.exists() else []
        except Exception:
            return []


def _save_memory(facts: list[dict]) -> None:
    with _memory_lock:
        MEMORY_FILE.parent.mkdir(parents=True, exist_ok=True)
        MEMORY_FILE.write_text(json.dumps(facts, indent=2))


def memory_remember(fact: str, tags: list[str] | None = None) -> dict:
    facts = _load_memory()
    entry = {"id": len(facts), "fact": fact, "tags": tags or [], "ts": time.time()}
    facts.append(entry)
    _save_memory(facts)
    return entry


def memory_recall(query: str, top_k: int = 5) -> list[dict]:
    facts = _load_memory()
    q = query.lower()
    scored = []
    for f in facts:
        text = f["fact"].lower()
        tags = " ".join(f.get("tags", [])).lower()
        score = sum(1 for w in q.split() if w in text or w in tags)
        if score > 0:
            scored.append((score, f))
    scored.sort(key=lambda x: -x[0])
    return [f for _, f in scored[:top_k]]


def memory_forget(fact_id: int) -> bool:
    facts = _load_memory()
    before = len(facts)
    facts = [f for f in facts if f.get("id") != fact_id]
    if len(facts) < before:
        _save_memory(facts)
        return True
    return False


# ── Dynamic ROS context injection ──────────────────────────────────────────────

_ros_context_cache: dict = {"text": "", "ts": 0}
_ROS_CONTEXT_TTL = 60.0


def _get_ros_context() -> str:
    now = time.time()
    if now - _ros_context_cache["ts"] < _ROS_CONTEXT_TTL and _ros_context_cache["text"]:
        return _ros_context_cache["text"]
    try:
        from roborun.ros_mcp import _discover
        disc = _discover()
        topics = disc.get("topics", [])[:25]
        nodes = disc.get("nodes", [])[:15]
        transports = disc.get("transports", {})
        parts = []
        if transports.get("dds") or transports.get("rosbridge"):
            parts.append(f"Transports: DDS={'yes' if transports.get('dds') else 'no'}, "
                         f"rosbridge={'yes' if transports.get('rosbridge') else 'no'}")
        if topics:
            topic_lines = [f"  {t['name']} [{t['type']}]" for t in topics]
            parts.append(f"Topics ({len(topics)}):\n" + "\n".join(topic_lines))
        if nodes:
            node_names = [n["name"] for n in nodes]
            parts.append(f"Nodes: {', '.join(node_names)}")
        text = "\n".join(parts)
    except Exception:
        text = ""
    _ros_context_cache.update(text=text, ts=now)
    return text


# ── SOUL.md identity ──────────────────────────────────────────────────────────

def _get_soul() -> str:
    if SOUL_FILE.exists():
        try:
            return SOUL_FILE.read_text().strip()
        except Exception:
            pass
    return ""


def _ensure_soul() -> None:
    if not SOUL_FILE.exists():
        SOUL_FILE.parent.mkdir(parents=True, exist_ok=True)
        SOUL_FILE.write_text(
            "# RoboRun Agent Identity\n\n"
            "1. **Safety first** — never exceed velocity limits, stop on uncertainty\n"
            "2. **Observe before acting** — check camera/sensors before physical actions\n"
            "3. **Verify after acting** — confirm actions succeeded via sensor feedback\n"
            "4. **Be concise** — this is a control panel, not a chatbot\n"
        )


_ensure_soul()

ROBOT_OPERATOR_CONTEXT = """You are the operator control agent for a robot running the dimOS stack.

## Architecture
- The robot runs **Daneel**, dimOS's built-in AI agent.
- You have MCP server connections to Daneel's live skill set (navigate, explore, speak, follow, patrol, etc.)
- These call the robot directly via dimOS's MCP server at port 9990.

## Key Skills
- `navigate_with_text` — go to a named place
- `begin_exploration` — autonomous frontier exploration
- `smart_follow_person` — follow people by description (YOLO+CLIP)
- `smart_follow_object` — follow objects by YOLO class
- `smart_find` — explore and search for something
- `query_scene` — what's visible right now
- `execute_sport_command` — RecoveryStand, FrontFlip, sit, etc.
- `speak` — robot speaks aloud
- `tag_location` — remember current location
- `where_am_i` — GPS and nearby landmarks

## Rules
1. Use skills for ALL physical actions — don't just describe, execute.
2. After commands, verify they worked.
3. Never claim the robot moved unless confirmed.
4. Be concise — this is a control panel.
"""


class RobotAgent:
    def __init__(self) -> None:
        self._proc: subprocess.Popen | None = None
        self._session_id: str | None = self._load_session()
        self._lock = threading.Lock()

    def _load_session(self) -> str | None:
        try:
            return SESSION_FILE.read_text().strip() or None
        except FileNotFoundError:
            return None

    def _save_session(self, sid: str) -> None:
        SESSION_FILE.parent.mkdir(parents=True, exist_ok=True)
        SESSION_FILE.write_text(sid)

    @property
    def is_alive(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def _start(self) -> None:
        mcp_config = json.dumps({
            "mcpServers": {
                "daneel": {
                    "type": "http",
                    "url": "http://127.0.0.1:9990/mcp",
                },
            }
        })
        cmd = [
            "claude",
            "--input-format", "stream-json",
            "--output-format", "stream-json",
            "--verbose",
            "--model", "claude-sonnet-4-6",
            "--permission-mode", "bypassPermissions",
            "--mcp-config", mcp_config,
        ]
        if self._session_id:
            cmd += ["--resume", self._session_id]
        self._proc = subprocess.Popen(
            cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            cwd=str(ROOT), bufsize=0,
        )

    def _ensure_started(self) -> None:
        if not self.is_alive:
            self._start()

    def stop(self) -> None:
        if self._proc:
            try:
                self._proc.terminate()
                self._proc.wait(timeout=3)
            except Exception:
                try: self._proc.kill()
                except Exception: pass
            self._proc = None

    def send(self, message: str) -> Iterator[dict]:
        with self._lock:
            try:
                self._ensure_started()
            except Exception as exc:
                yield {"type": "error", "error": f"Failed to start agent: {exc}"}
                return

            if self._session_id is None:
                instructed = f"{ROBOT_OPERATOR_CONTEXT}\n\nOperator request:\n{message}"
            else:
                instructed = message

            msg_json = json.dumps({
                "type": "user",
                "message": {"role": "user", "content": [{"type": "text", "text": instructed}]},
            }) + "\n"

            try:
                self._proc.stdin.write(msg_json.encode())
                self._proc.stdin.flush()
            except OSError as exc:
                self._proc = None
                yield {"type": "error", "error": f"Write failed: {exc}"}
                return

            accumulated = ""
            while True:
                try:
                    raw = self._proc.stdout.readline()
                except OSError:
                    break
                if not raw:
                    yield {"type": "error", "error": "Agent process ended"}
                    self._proc = None
                    return
                try:
                    event = json.loads(raw.decode())
                except json.JSONDecodeError:
                    continue

                etype = event.get("type")
                if etype == "system":
                    sid = event.get("session_id")
                    if sid:
                        self._session_id = sid
                        self._save_session(sid)
                elif etype == "assistant":
                    for block in event.get("message", {}).get("content", []):
                        btype = block.get("type")
                        if btype == "text":
                            chunk = block.get("text", "")
                            accumulated += chunk
                            yield {"type": "text", "text": chunk, "accumulated": accumulated}
                        elif btype == "tool_use":
                            yield {"type": "tool_use", "tool_id": block.get("id", ""),
                                   "tool_name": block.get("name", ""), "tool_input": block.get("input", {})}
                        elif btype == "thinking":
                            thinking = block.get("thinking", "")
                            if thinking:
                                yield {"type": "thinking", "thinking": thinking}
                elif etype == "user":
                    for block in event.get("message", {}).get("content", []):
                        if isinstance(block, dict) and block.get("type") == "tool_result":
                            content = block.get("content", [])
                            result_text = ""
                            if isinstance(content, list):
                                for c in content:
                                    if isinstance(c, dict) and c.get("type") == "text":
                                        result_text += c.get("text", "")
                            elif isinstance(content, str):
                                result_text = content
                            yield {"type": "tool_result", "tool_use_id": block.get("tool_use_id", ""),
                                   "result": result_text[:1200], "is_error": block.get("is_error", False)}
                elif etype == "result":
                    final = accumulated or event.get("result", "")
                    yield {"type": "done", "text": final, "cost": event.get("total_cost_usd", 0.0),
                           "error": event.get("result") if event.get("is_error") else None}
                    return

    def clear_session(self) -> None:
        self.stop()
        self._session_id = None
        try: SESSION_FILE.unlink()
        except FileNotFoundError: pass


# ── Fast agent (direct Anthropic SDK, sensor pre-injection) ───────────────────

# Frame paths — must match server.py constants
_FRAME_PATHS = [
    Path("/tmp/go2_hackathon_frame.jpg"),
    Path("/tmp/roborun_frame.jpg"),
    Path("/tmp/go2_camera_frame.jpg"),
]
_STATE_PATHS = [
    Path("/tmp/go2_hackathon_state.json"),
    Path("/tmp/roborun_state.json"),
]
_MAX_FRAME_AGE = 3.0  # seconds

_FAST_SYSTEM = """You are a robot operator. The current camera frame and YOLO detections are injected into every message — you can see the robot's live view directly without calling any tools.

Use tools ONLY for physical actions or memory lookups:
- execute_skill: high-level behaviors (navigate, explore, follow, find, speak, etc.)
- move: direct velocity command — fast, no round-trip, use for nudges and short moves
- memory_search: search past observations by text

Do NOT call query_scene or any perception tool — you already have the live frame.
After actions, state what changed based on the updated frame in the next turn.
Be concise."""

_FAST_TOOLS = [
    {
        "name": "execute_skill",
        "description": "Execute a dimOS robot skill. Handles navigation, exploration, following, finding, speaking, sport commands, and GPS.",
        "input_schema": {
            "type": "object",
            "properties": {
                "skill": {
                    "type": "string",
                    "description": "One of: navigate_with_text, begin_exploration, smart_follow_person, smart_follow_object, smart_find, query_scene, execute_sport_command, speak, tag_location, where_am_i",
                },
                "args": {
                    "type": "object",
                    "description": "Skill-specific arguments",
                },
            },
            "required": ["skill"],
        },
    },
    {
        "name": "move",
        "description": "Send a direct velocity command to the robot (/cmd_vel). Fast — no MCP round-trip. linear_x: forward (+) / back (-). angular_z: left (+) / right (-).",
        "input_schema": {
            "type": "object",
            "properties": {
                "linear_x": {"type": "number", "description": "m/s, e.g. 0.3"},
                "angular_z": {"type": "number", "description": "rad/s, e.g. 0.5"},
                "duration_s": {"type": "number", "description": "Seconds to hold command, then stop (0 = one-shot)"},
            },
            "required": [],
        },
    },
    {
        "name": "memory_search",
        "description": "Search the robot's spatial memory for past observations using CLIP semantic search.",
        "input_schema": {
            "type": "object",
            "properties": {
                "query": {"type": "string", "description": "What to search for e.g. 'red mug', 'charging dock'"},
                "top_k": {"type": "integer", "description": "Max results (default 5)"},
            },
            "required": ["query"],
        },
    },
    {
        "name": "get_telemetry",
        "description": "Get current robot telemetry — battery, position, orientation, velocity, joint states.",
        "input_schema": {"type": "object", "properties": {}},
    },
    {
        "name": "get_trajectory",
        "description": "Get the recorded trajectory as timestamped poses.",
        "input_schema": {
            "type": "object",
            "properties": {"limit": {"type": "integer", "description": "Max points"}},
        },
    },
    {
        "name": "takeoff",
        "description": "Arm and take off to specified altitude (drone only).",
        "input_schema": {
            "type": "object",
            "properties": {"altitude": {"type": "number"}},
        },
    },
    {
        "name": "land",
        "description": "Land the drone at current position.",
        "input_schema": {"type": "object", "properties": {}},
    },
    {
        "name": "goto_waypoint",
        "description": "Fly to a 3D waypoint (drone only).",
        "input_schema": {
            "type": "object",
            "properties": {
                "x": {"type": "number"}, "y": {"type": "number"}, "z": {"type": "number"},
            },
            "required": ["x", "y", "z"],
        },
    },
    {
        "name": "remember",
        "description": "Store a fact in persistent memory. Facts persist across sessions and are shared between agents.",
        "input_schema": {
            "type": "object",
            "properties": {
                "fact": {"type": "string", "description": "The fact to remember"},
                "tags": {"type": "array", "items": {"type": "string"}, "description": "Tags for retrieval"},
            },
            "required": ["fact"],
        },
    },
    {
        "name": "recall",
        "description": "Search persistent memory for relevant facts by keyword.",
        "input_schema": {
            "type": "object",
            "properties": {
                "query": {"type": "string", "description": "Search query"},
                "top_k": {"type": "integer", "description": "Max results (default 5)"},
            },
            "required": ["query"],
        },
    },
    {
        "name": "forget",
        "description": "Delete a fact from persistent memory by ID.",
        "input_schema": {
            "type": "object",
            "properties": {
                "id": {"type": "integer", "description": "Fact ID to delete"},
            },
            "required": ["id"],
        },
    },
    {
        "name": "find_object",
        "description": "Actively search for an object by rotating the robot and checking the camera at each step. Returns when found or after full rotation.",
        "input_schema": {
            "type": "object",
            "properties": {
                "label": {"type": "string", "description": "Object label to find (e.g. 'person', 'cup', 'chair')"},
                "max_rotation_deg": {"type": "number", "description": "Max rotation before giving up (default 360)"},
            },
            "required": ["label"],
        },
    },
]


def _read_sensor_context() -> list[dict]:
    """Return multimodal content blocks with live camera + YOLO state."""
    import base64
    content: list[dict] = []

    # Camera frame (freshest available)
    for p in _FRAME_PATHS:
        try:
            if p.exists() and (time.time() - p.stat().st_mtime) < _MAX_FRAME_AGE:
                data = p.read_bytes()
                content.append({
                    "type": "image",
                    "source": {"type": "base64", "media_type": "image/jpeg",
                               "data": base64.b64encode(data).decode()},
                })
                break
        except Exception:
            pass

    # YOLO detections / robot state
    for p in _STATE_PATHS:
        try:
            if p.exists() and (time.time() - p.stat().st_mtime) < _MAX_FRAME_AGE:
                state = json.loads(p.read_text())
                dets = state.get("detections", [])
                parts = []
                if dets:
                    labels = [f"{d.get('label','?')}({d.get('confidence',0):.2f})" for d in dets[:12]]
                    parts.append(f"YOLO: {', '.join(labels)}")
                pose = state.get("pose") or state.get("position")
                if pose:
                    parts.append(f"pose: {pose}")
                if parts:
                    content.append({"type": "text", "text": "[" + " | ".join(parts) + "]"})
                break
        except Exception:
            pass

    return content


def _execute_fast_tool(name: str, args: dict) -> str:
    if name == "execute_skill":
        skill = args.get("skill", "")
        skill_args = args.get("args", {})
        result = _call_local_api("/api/mcp/call", {"name": skill, "args": skill_args})
        if not result.get("ok"):
            return f"Skill dispatch failed: {result.get('error')}"
        task_id = result.get("task_id", "")
        for _ in range(60):  # up to 30s
            time.sleep(0.5)
            poll = _call_local_api(f"/api/mcp/result?id={task_id}", method="GET")
            status = poll.get("status", "pending")
            if status == "done":
                return poll.get("result", "Done.")
            if status == "error":
                return f"Skill error: {poll.get('error')}"
        return "Skill timed out."

    elif name == "move":
        lx, _, az = _clamp_velocity(
            float(args.get("linear_x", 0.0)), 0.0, float(args.get("angular_z", 0.0)))
        dur = float(args.get("duration_s", 0.0))
        result = _call_local_api("/api/ros/move", {"linear_x": lx, "angular_z": az})
        if dur > 0:
            time.sleep(min(dur, 5.0))
            _call_local_api("/api/ros/move", {"linear_x": 0.0, "angular_z": 0.0})
        return "Move sent." if result.get("ok") else f"Move failed: {result.get('error')}"

    elif name == "memory_search":
        result = _call_local_api("/api/memory/search", {
            "query": args.get("query", ""),
            "mode": "clip",
            "top_k": int(args.get("top_k", 5)),
        })
        if result.get("ok"):
            records = result.get("results", [])
            if not records:
                return "No matching memories."
            lines = []
            for r in records:
                loc = f"({r.get('x','?'):.1f}, {r.get('y','?'):.1f})" if r.get("x") is not None else "?"
                dets = r.get("detections") or []
                if isinstance(dets, str):
                    try: dets = json.loads(dets)
                    except Exception: dets = []
                labels = [d.get("label", "") for d in dets[:4] if isinstance(d, dict)]
                lines.append(f"{loc} — {', '.join(labels) or 'no labels'}")
            return "\n".join(lines)
        return f"Search failed: {result.get('error')}"

    elif name == "get_telemetry":
        result = _call_local_api("/api/telemetry", method="GET")
        if result.get("ok"):
            return json.dumps(result.get("telemetry", {}), indent=2)
        return "No telemetry available"

    elif name == "get_trajectory":
        limit = int(args.get("limit", 500))
        result = _call_local_api(f"/api/trajectory?limit={limit}", method="GET")
        traj = result.get("trajectory", [])
        return f"{len(traj)} points" if traj else "No trajectory data"

    elif name == "takeoff":
        alt = float(args.get("altitude", 2.0))
        result = _call_local_api("/api/sim/altitude", {"altitude": alt})
        return f"Takeoff to {alt}m" if result.get("ok") else f"Failed: {result.get('error')}"

    elif name == "land":
        result = _call_local_api("/api/sim/altitude", {"altitude": 0.1})
        return "Landing" if result.get("ok") else f"Failed: {result.get('error')}"

    elif name == "goto_waypoint":
        x, y, z = float(args.get("x", 0)), float(args.get("y", 0)), float(args.get("z", 2))
        result = _call_local_api("/api/sim/waypoint", {"x": x, "y": y, "z": z})
        return f"Waypoint ({x},{y},{z})" if result.get("ok") else f"Failed: {result.get('error')}"

    elif name == "remember":
        entry = memory_remember(args.get("fact", ""), args.get("tags"))
        return f"Remembered (id={entry['id']}): {entry['fact'][:100]}"

    elif name == "recall":
        results = memory_recall(args.get("query", ""), int(args.get("top_k", 5)))
        if not results:
            return "No matching facts in memory."
        return "\n".join(f"[{f['id']}] {f['fact']}" for f in results)

    elif name == "forget":
        ok = memory_forget(int(args.get("id", -1)))
        return "Fact deleted." if ok else "Fact not found."

    elif name == "find_object":
        return _find_object(args.get("label", ""), float(args.get("max_rotation_deg", 360)))

    return f"Unknown tool: {name}"


def _find_object(label: str, max_rotation_deg: float = 360) -> str:
    """Rotate the robot incrementally, checking camera at each step."""
    import math
    step_deg = 30
    steps = int(max_rotation_deg / step_deg)
    label_lower = label.lower()

    for i in range(steps):
        # Check camera + state
        for p in _STATE_PATHS:
            try:
                if p.exists() and (time.time() - p.stat().st_mtime) < 5.0:
                    state = json.loads(p.read_text())
                    for det in state.get("detections", []):
                        if det.get("label", "").lower() == label_lower and det.get("confidence", 0) >= 0.4:
                            bbox = det.get("bbox", [])
                            cx = (bbox[0] + bbox[2]) / 2 if len(bbox) >= 4 else 0
                            return (f"Found '{label}' at step {i} ({i*step_deg}deg rotation). "
                                    f"Confidence: {det['confidence']:.2f}, "
                                    f"bbox center x: {cx:.0f}")
            except Exception:
                pass

        # Rotate by step_deg
        az = math.radians(step_deg)
        lx, _, az_clamped = _clamp_velocity(0, 0, az)
        _call_local_api("/api/ros/move", {"linear_x": 0, "angular_z": az_clamped})
        time.sleep(0.8)
        _call_local_api("/api/ros/move", {"linear_x": 0, "angular_z": 0})
        time.sleep(0.5)

    return f"'{label}' not found after {max_rotation_deg}deg rotation."


class FastRobotAgent:
    """Direct Anthropic SDK agent with sensor pre-injection.

    Eliminates subprocess + MCP round-trip for perception.
    Camera frame and YOLO state are injected as multimodal content before
    every message, so Claude never needs a tool call to see current state.
    Tools are only invoked for physical actions and memory lookups.

    Requires ANTHROPIC_API_KEY in environment.
    """

    MODEL = "claude-sonnet-4-6"
    MAX_TOOL_ROUNDS = 6

    def __init__(self) -> None:
        self._client = None
        self._history: list[dict] = []
        self._lock = threading.Lock()

    def _get_client(self):
        if self._client is None:
            import anthropic
            import os
            key = os.environ.get("ANTHROPIC_API_KEY", "")
            if not key:
                raise RuntimeError("ANTHROPIC_API_KEY not set")
            self._client = anthropic.Anthropic(api_key=key)
        return self._client

    @property
    def is_alive(self) -> bool:
        return True  # stateless SDK, always alive

    def send(self, message: str) -> Iterator[dict]:
        with self._lock:
            try:
                client = self._get_client()
            except Exception as exc:
                yield {"type": "error", "error": str(exc)}
                return

            # Pre-inject sensor context into the user message
            sensor = _read_sensor_context()
            user_content = sensor + [{"type": "text", "text": message}]
            self._history.append({"role": "user", "content": user_content})

            accumulated = ""

            for _round in range(self.MAX_TOOL_ROUNDS):
                # Build dynamic system prompt with ROS context + SOUL + memory
                system_parts = [_FAST_SYSTEM]
                soul = _get_soul()
                if soul:
                    system_parts.append(f"\n## Identity\n{soul}")
                ros_ctx = _get_ros_context()
                if ros_ctx:
                    system_parts.append(f"\n## Live ROS Graph\n{ros_ctx}")
                recent_mem = memory_recall("recent", top_k=5)
                if recent_mem:
                    mem_lines = [f"- {f['fact']}" for f in recent_mem]
                    system_parts.append(f"\n## Memory\n" + "\n".join(mem_lines))
                dynamic_system = "\n".join(system_parts)

                # Stream the response
                try:
                    with client.messages.stream(
                        model=self.MODEL,
                        max_tokens=2048,
                        system=dynamic_system,
                        messages=self._history,
                        tools=_FAST_TOOLS,
                    ) as stream:
                        for event in stream:
                            if (event.type == "content_block_delta"
                                    and hasattr(event.delta, "text")):
                                chunk = event.delta.text
                                accumulated += chunk
                                yield {"type": "text", "text": chunk,
                                       "accumulated": accumulated}
                        final = stream.get_final_message()
                except Exception as exc:
                    yield {"type": "error", "error": f"API error: {exc}"}
                    return

                # Add assistant turn to history
                self._history.append({
                    "role": "assistant",
                    "content": [b.model_dump() for b in final.content],
                })

                # Check stop reason
                if final.stop_reason != "tool_use":
                    break

                # Execute all tool calls
                tool_results = []
                for block in final.content:
                    if block.type != "tool_use":
                        continue
                    yield {"type": "tool_use", "tool_name": block.name,
                           "tool_input": block.input}
                    result_text = _execute_fast_tool(block.name, block.input)
                    yield {"type": "tool_result", "tool_name": block.name,
                           "result": result_text[:1200]}
                    tool_results.append({
                        "type": "tool_result",
                        "tool_use_id": block.id,
                        "content": result_text,
                    })

                # Append tool results and loop for next round
                self._history.append({"role": "user", "content": tool_results})

            yield {"type": "done", "result": accumulated}

    def clear_session(self) -> None:
        with self._lock:
            self._history = []

    def stop(self) -> None:
        pass  # no subprocess to stop


# ── Gemini agent ──────────────────────────────────────────────────────────────

_GEMINI_SYSTEM = """You are the operator control agent for a robot. You have tools to:
- capture a camera frame and describe what the robot sees
- move the robot (forward/back/turn)
- call any dimOS skill (navigate, explore, follow, find, speak, patrol, etc.)
- search the robot's spatial memory for past observations
- publish to ROS 2 topics directly

Use tools for ALL physical actions. After each command, verify it worked.
Be concise — this is a live control panel."""

# Tool declarations matching agenticROS's MCP surface
_GEMINI_TOOLS = [
    {
        "name": "camera_snapshot",
        "description": "Capture a single frame from the robot camera. Returns a JPEG image for visual inspection.",
        "parameters": {
            "type": "object",
            "properties": {},
        },
    },
    {
        "name": "move_robot",
        "description": "Send velocity command to the robot. Positive linear_x = forward, negative = backward. Positive angular_z = turn left, negative = turn right.",
        "parameters": {
            "type": "object",
            "properties": {
                "linear_x": {"type": "number", "description": "Forward/backward speed in m/s (e.g. 0.3)"},
                "linear_y": {"type": "number", "description": "Lateral speed in m/s (for holonomic robots)"},
                "angular_z": {"type": "number", "description": "Rotation speed in rad/s"},
                "topic": {"type": "string", "description": "ROS topic (default /cmd_vel)"},
            },
            "required": [],
        },
    },
    {
        "name": "ros_publish",
        "description": "Publish a message to any ROS 2 topic.",
        "parameters": {
            "type": "object",
            "properties": {
                "topic": {"type": "string"},
                "type": {"type": "string", "description": "ROS message type e.g. geometry_msgs/Twist"},
                "message": {"type": "object"},
            },
            "required": ["topic", "type", "message"],
        },
    },
    {
        "name": "ros_subscribe_once",
        "description": "Read a single message from any ROS 2 topic and return it.",
        "parameters": {
            "type": "object",
            "properties": {
                "topic": {"type": "string"},
                "type": {"type": "string"},
                "timeout_ms": {"type": "number", "description": "Timeout in milliseconds (default 5000)"},
            },
            "required": ["topic"],
        },
    },
    {
        "name": "ros_list_topics",
        "description": "List all available ROS 2 topics and their types.",
        "parameters": {
            "type": "object",
            "properties": {},
        },
    },
    {
        "name": "call_dimos_skill",
        "description": "Call a dimOS robot skill directly. Skills: navigate_with_text, begin_exploration, smart_follow_person, smart_follow_object, smart_find, query_scene, execute_sport_command, speak, tag_location, where_am_i.",
        "parameters": {
            "type": "object",
            "properties": {
                "skill": {"type": "string", "description": "Skill name e.g. navigate_with_text"},
                "args": {"type": "object", "description": "Skill arguments"},
            },
            "required": ["skill"],
        },
    },
    {
        "name": "memory_search",
        "description": "Search the robot's spatial memory by text (CLIP semantic search). Returns matching past observations with locations.",
        "parameters": {
            "type": "object",
            "properties": {
                "query": {"type": "string", "description": "What to search for e.g. 'red mug', 'person in hallway'"},
                "top_k": {"type": "integer", "description": "Max results (default 5)"},
            },
            "required": ["query"],
        },
    },
    {
        "name": "memory_search_nearby",
        "description": "Search the robot's spatial memory for observations near given coordinates.",
        "parameters": {
            "type": "object",
            "properties": {
                "x": {"type": "number"},
                "y": {"type": "number"},
                "radius": {"type": "number", "description": "Search radius in meters (default 2.0)"},
            },
            "required": ["x", "y"],
        },
    },
]


def _call_local_api(path: str, payload: dict | None = None, method: str = "POST") -> dict:
    url = f"http://127.0.0.1:8765{path}"
    body = json.dumps(payload or {}).encode() if payload is not None else b"{}"
    req = urllib.request.Request(url, data=body,
                                  headers={"Content-Type": "application/json"}, method=method)
    try:
        with urllib.request.urlopen(req, timeout=15) as resp:
            return json.loads(resp.read().decode())
    except Exception as exc:
        return {"ok": False, "error": str(exc)}


def _execute_gemini_tool(name: str, args: dict) -> str:
    if name == "camera_snapshot":
        result = _call_local_api("/api/camera", {}, method="GET")
        if result.get("ok") and result.get("image"):
            return "Camera frame captured (base64 JPEG available). Robot camera is active."
        return f"Camera unavailable: {result.get('error', 'no frame')}"

    elif name == "move_robot":
        lx, ly, az = _clamp_velocity(
            float(args.get("linear_x", 0.0)),
            float(args.get("linear_y", 0.0)),
            float(args.get("angular_z", 0.0)))
        result = _call_local_api("/api/ros/move", {
            "linear_x": lx, "linear_y": ly, "angular_z": az,
            "topic": args.get("topic", "/cmd_vel"),
        })
        return "Move command sent." if result.get("ok") else f"Move failed: {result.get('error')}"

    elif name == "ros_publish":
        result = _call_local_api("/api/ros/publish", {
            "topic": args["topic"],
            "type": args["type"],
            "message": args["message"],
        })
        return "Published." if result.get("ok") else f"Publish failed: {result.get('error')}"

    elif name == "ros_subscribe_once":
        result = _call_local_api("/api/ros/subscribe-once", {
            "topic": args["topic"],
            "type": args.get("type", ""),
            "timeout": args.get("timeout_ms", 5000),
        })
        if result.get("ok"):
            msg = result.get("message")
            return json.dumps(msg) if msg else "No message received within timeout."
        return f"Subscribe failed: {result.get('error')}"

    elif name == "ros_list_topics":
        result = _call_local_api("/api/ros/topics", method="GET")
        if result.get("ok"):
            topics = result.get("topics", [])
            return "\n".join(f"{t['topic']} [{t['type']}]" for t in topics[:30]) or "No topics found."
        return f"Failed: {result.get('error')}"

    elif name == "call_dimos_skill":
        result = _call_local_api("/api/mcp/call", {
            "name": args["skill"],
            "args": args.get("args", {}),
        })
        if result.get("ok"):
            task_id = result.get("task_id", "")
            # poll for result (max 15s)
            for _ in range(30):
                time.sleep(0.5)
                poll = _call_local_api(f"/api/mcp/result?id={task_id}", method="GET")
                status = poll.get("status", "pending")
                if status == "done":
                    return poll.get("result", "Done.")
                elif status == "error":
                    return f"Skill error: {poll.get('error')}"
            return "Skill timed out."
        return f"MCP call failed: {result.get('error')}"

    elif name == "memory_search":
        result = _call_local_api("/api/memory/search", {
            "query": args["query"],
            "top_k": args.get("top_k", 5),
        })
        if result.get("ok"):
            memories = result.get("memories", [])
            if not memories:
                return "No matching memories found."
            lines = []
            for m in memories:
                loc = f"({m.get('x','?')}, {m.get('y','?')})" if m.get("x") is not None else "unknown location"
                dets = m.get("detections", [])
                labels = [d.get("label", "") for d in (dets if isinstance(dets, list) else [])]
                lines.append(f"[{m.get('ts','?')}] at {loc}: {', '.join(labels) or 'no detections'}")
            return "\n".join(lines)
        return f"Memory search failed: {result.get('error')}"

    elif name == "memory_search_nearby":
        result = _call_local_api("/api/memory/search", {
            "mode": "nearby",
            "x": args["x"],
            "y": args["y"],
            "radius": args.get("radius", 2.0),
            "top_k": 10,
        })
        if result.get("ok"):
            return json.dumps(result.get("memories", []))
        return f"Failed: {result.get('error')}"

    return f"Unknown tool: {name}"


class GeminiAgent:
    """Gemini robot agent using function calling — no MCP required.

    Exposes the same robot tool surface as agenticROS's MCP tool definitions.
    Requires: pip install google-generativeai
    Set GEMINI_API_KEY in environment.
    """

    def __init__(self, model: str = "gemini-2.0-flash") -> None:
        self._model_name = model
        self._model = None
        self._history: list[dict] = []
        self._lock = threading.Lock()

    def _ensure_loaded(self) -> None:
        if self._model is not None:
            return
        import google.generativeai as genai
        import os
        api_key = os.environ.get("GEMINI_API_KEY", "")
        if not api_key:
            raise RuntimeError("GEMINI_API_KEY environment variable not set")
        genai.configure(api_key=api_key)
        tool_defs = [{"function_declarations": _GEMINI_TOOLS}]
        self._model = genai.GenerativeModel(
            model_name=self._model_name,
            system_instruction=_GEMINI_SYSTEM,
            tools=tool_defs,
        )
        self._chat = self._model.start_chat(history=self._history)

    def send(self, message: str) -> Iterator[dict]:
        with self._lock:
            try:
                self._ensure_loaded()
            except Exception as exc:
                yield {"type": "error", "error": str(exc)}
                return

            try:
                response = self._chat.send_message(message)
            except Exception as exc:
                yield {"type": "error", "error": f"Gemini API error: {exc}"}
                return

            # agentic loop: execute tool calls until final text
            MAX_ROUNDS = 8
            for _ in range(MAX_ROUNDS):
                tool_calls = []
                text_parts = []

                for part in response.parts:
                    if hasattr(part, "function_call") and part.function_call.name:
                        tool_calls.append(part.function_call)
                    elif hasattr(part, "text") and part.text:
                        text_parts.append(part.text)

                for tc in tool_calls:
                    args = dict(tc.args) if tc.args else {}
                    yield {"type": "tool_use", "tool_name": tc.name, "tool_input": args}
                    result_text = _execute_gemini_tool(tc.name, args)
                    yield {"type": "tool_result", "tool_name": tc.name, "result": result_text[:1200]}

                    try:
                        import google.generativeai as genai
                        tool_response = genai.protos.Part(
                            function_response=genai.protos.FunctionResponse(
                                name=tc.name,
                                response={"result": result_text},
                            )
                        )
                        response = self._chat.send_message(tool_response)
                    except Exception as exc:
                        yield {"type": "error", "error": f"Tool response error: {exc}"}
                        return

                if text_parts:
                    text = "".join(text_parts)
                    yield {"type": "text", "text": text, "accumulated": text}
                    yield {"type": "done", "result": text}
                    return

                if not tool_calls:
                    yield {"type": "done", "result": ""}
                    return

            yield {"type": "error", "error": "Agent loop limit reached"}

    def clear_session(self) -> None:
        with self._lock:
            self._history = []
            if self._model is not None:
                import google.generativeai as genai
                self._chat = self._model.start_chat(history=[])

    @property
    def is_alive(self) -> bool:
        return self._model is not None
