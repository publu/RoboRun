"""The flight deck's in-process agent (command bar → /api/agent/chat).

One implementation: an agentic loop with sensor pre-injection — the
current camera frame and YOLO detections are attached to every message,
so the model sees live state without a tool round-trip. Tools cover
physical actions and memory. Velocity-clamped like everything else.

This is the *in-process* agent for the deck's command bar. The primary
way to drive RoboRun with an LLM remains the MCP server (Claude Code /
Cursor / any client connects to us, not the other way around). The model
follows the "smart" tier (ROBORUN_MODEL_SMART) when it resolves to an
Anthropic model; the agent loop currently requires the anthropic SDK.
"""
from __future__ import annotations

import json
import logging
import os
import threading
import time
import urllib.request
from pathlib import Path
from typing import Iterator

log = logging.getLogger(__name__)

ROOT = Path(__file__).resolve().parent.parent
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


# ── Fast agent (direct Anthropic SDK, sensor pre-injection) ───────────────────

_FRAME_PATHS = [
    Path("/tmp/roborun_frame.jpg"),
    Path("/tmp/roborun_camera.jpg"),
]
_STATE_PATHS = [
    Path("/tmp/roborun_state.json"),
]
_MAX_FRAME_AGE = 3.0  # seconds

_FAST_SYSTEM = """You are a robot operator powered by RoboRun. The current camera frame and YOLO detections are injected into every message — you can see the robot's live view directly.

Use tools for physical actions and memory:
- move: direct velocity command, fast, use for nudges and short moves
- find_object: rotate and search for objects by label
- memory_search: search past observations by text
- remember/recall/forget: persistent cross-agent memory
- get_telemetry: battery, position, velocity, joints

You already have the live camera frame — don't call perception tools redundantly.
After actions, verify via the updated frame in the next turn. Be concise.

## Authoring behaviors (the real superpower)
move only nudges the robot once. The lasting way to give it a skill is to WRITE A
BEHAVIOR: a small Python control loop that runs at 10 Hz with no LLM in the loop.
Prefer authoring a behavior for any standing task ("patrol", "follow me", "back off
from walls"); use move only for one-off pokes. The closed loop you run:

  1. write_behavior(name, source)  → creates behaviors/<name>.py, hot-reloads and
     starts within ~1s. The tool reports back whether it loaded and any error.
  2. arena_status + read_timeline  → watch what it actually did (pose, detections,
     anything it logged, and crashes). This is how you debug "why did it fail".
  3. read_behavior → write_behavior again to fix it. Iterate until it works.
  4. set_behavior(name, "disable") to stop it; "enable" to resume.
  list_behaviors shows everything running, its run/error counts and last error.

A behavior is ONE file. Use exactly this handle — nothing else is portable:

    from roborun.behaviors import behavior

    @behavior(hz=10)              # control loop; or @behavior(every=5.0) for slow loops
    def patrol(robot):
        people = robot.see("person")         # list of {cx,cy,w,h,label,conf,dist}
        if people:
            robot.say("person spotted")
            return robot.stop()
        ahead = robot.lidar()[0]             # 36 floats, meters, [0] = straight ahead
        if ahead < 0.8:
            return robot.move(turn=1.0)      # too close to a wall, turn
        robot.move(forward=0.4)              # forward / strafe / turn / climb, each in [-1,1]

Handle reference (the whole contract — same file runs sim → real robot):
  robot.see(label) · robot.move(forward,strafe,turn,climb) · robot.stop()
  robot.lidar() → 36 floats · robot.pose() → {x,z,heading} · robot.say(t)/robot.log(t)
  robot.remember(k,v)/robot.recall(k) · robot.state (dict persists across ticks)
  robot.goto(x,z) · robot.approach(thing) · robot.seen(label)
Keep behaviors short and readable — a person should grasp the file in ten seconds.

## Triaging from the eval scoreboard
Behaviors are scored by *scenarios* (named runs with pass/fail + metrics), grouped
into *suites* with a pass-rate. When asked to improve the robot, start from the
scoreboard, don't guess: list_suites → find the lowest pass-rate → list_scenarios
(suite=…, outcome="failed") to see which scenarios fail and why → read_behavior on
the behavior behind it → write_behavior to fix → re-observe. That's the same
"reproduce, find the failure mode, fix, re-validate" loop a robotics engineer runs,
but at software speed."""

_FAST_TOOLS = [
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
    {
        "name": "list_behaviors",
        "description": "List every behavior currently loaded — name, file, whether it's running (enabled), tick count, error count, and last error. Use this to see what skills the robot has before writing or editing one.",
        "input_schema": {"type": "object", "properties": {}},
    },
    {
        "name": "read_behavior",
        "description": "Read the Python source of a behavior so you can edit it. Pass the behavior name (or file stem).",
        "input_schema": {
            "type": "object",
            "properties": {"name": {"type": "string", "description": "Behavior name / file stem, e.g. 'patrol'"}},
            "required": ["name"],
        },
    },
    {
        "name": "write_behavior",
        "description": "Create or overwrite behaviors/<name>.py with a full @behavior-decorated control loop. It hot-reloads and starts within ~1s; this tool waits and reports back whether it loaded and any runtime error so you can iterate. This is how you give the robot a lasting skill from natural language.",
        "input_schema": {
            "type": "object",
            "properties": {
                "name": {"type": "string", "description": "snake_case file stem, e.g. 'perimeter_patrol'"},
                "source": {"type": "string", "description": "Complete Python source: 'from roborun.behaviors import behavior' + an @behavior(hz=...) function taking (robot)."},
            },
            "required": ["name", "source"],
        },
    },
    {
        "name": "set_behavior",
        "description": "Start (enable) or stop (disable) a loaded behavior by name. Disabling stops the robot and halts that loop; enabling resumes it.",
        "input_schema": {
            "type": "object",
            "properties": {
                "name": {"type": "string"},
                "action": {"type": "string", "enum": ["enable", "disable"]},
            },
            "required": ["name", "action"],
        },
    },
    {
        "name": "read_timeline",
        "description": "Read the most recent events from the run timeline — behavior loads/errors, things the robot said or logged, detections, agent actions. This is how you observe what a behavior actually did and debug why it failed.",
        "input_schema": {
            "type": "object",
            "properties": {
                "limit": {"type": "integer", "description": "How many recent events (default 25)"},
                "contains": {"type": "string", "description": "Optional: only events whose source or title contains this substring (e.g. a behavior name)"},
            },
        },
    },
    {
        "name": "arena_status",
        "description": "Get the sim's ground-truth state — whether the arena is open, the robot's pose, the current level, and occlusion-checked detections. Use it to verify a behavior is moving the robot the way you intended.",
        "input_schema": {"type": "object", "properties": {}},
    },
    {
        "name": "list_scenarios",
        "description": "List recent scored scenario runs — name, outcome (passed/failed/error), suite, tags, key metrics. This is the eval record: how the robot's behaviors have actually performed. Filter by suite, outcome, or scenario name.",
        "input_schema": {
            "type": "object",
            "properties": {
                "limit": {"type": "integer", "description": "Max rows (default 15)"},
                "suite": {"type": "string"},
                "outcome": {"type": "string", "enum": ["passed", "failed", "error"]},
                "name": {"type": "string"},
            },
        },
    },
    {
        "name": "list_suites",
        "description": "List scenario suites with their pass-rate, run count, and latest activity — the scoreboard of how each capability is doing. Start here to find what's failing, then list_scenarios(suite=…) to drill in, then read_behavior / write_behavior to fix the behavior behind it.",
        "input_schema": {"type": "object", "properties": {}},
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
    if name == "move":
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

    elif name == "list_behaviors":
        from roborun.behaviors import BehaviorRunner
        rows = BehaviorRunner.get().statuses()
        if not rows:
            return "No behaviors loaded. Write one with write_behavior."
        lines = []
        for s in rows:
            run = "running" if s.get("enabled") else "stopped"
            tail = f" · last_error: {s['last_error']}" if s.get("last_error") else ""
            lines.append(f"{s['name']} [{run}] {Path(s['file']).name} "
                         f"· {s.get('runs',0)} ticks, {s.get('errors',0)} errors{tail}")
        return "\n".join(lines)

    elif name == "read_behavior":
        return _read_behavior_source(str(args.get("name", "")))

    elif name == "write_behavior":
        from roborun.behaviors import write_behavior_file
        bname = str(args.get("name", ""))
        result = write_behavior_file(bname, str(args.get("source", "")))
        if not result.get("ok"):
            return f"Not written: {result.get('error')}"
        # Give hot reload a beat, then report whether it actually loaded + ran.
        time.sleep(1.4)
        return f"Wrote {result['path']}. " + _behavior_health(bname)

    elif name == "set_behavior":
        from roborun.behaviors import BehaviorRunner
        bname = str(args.get("name", "")).strip()
        action = str(args.get("action", "enable"))
        ok = BehaviorRunner.get().set_enabled(bname, action == "enable")
        return (f"{bname} {action}d." if ok else f"No behavior named {bname!r}.")

    elif name == "read_timeline":
        from roborun.events import recent
        limit = int(args.get("limit", 25))
        contains = str(args.get("contains", "")).lower().strip()
        rows = recent(limit if not contains else 200)
        if contains:
            rows = [e for e in rows
                    if contains in str(e.get("source", "")).lower()
                    or contains in str(e.get("title", "")).lower()][-limit:]
        if not rows:
            return "Timeline empty (no matching events)."
        return "\n".join(f"[{e.get('type','?')}/{e.get('source','?')}] {e.get('title','')}"
                         for e in rows)

    elif name == "arena_status":
        from roborun.ros_mcp import _tool_arena_status
        r = _tool_arena_status({})
        if not r.get("active"):
            return r.get("hint", "Arena not open.")
        dets = r.get("detections") or []
        labels = ", ".join(sorted({d.get("label", "?") for d in dets})) or "none"
        return (f"Arena live · level={r.get('level')} · pose={r.get('pose')} · "
                f"sees: {labels}")

    elif name == "list_scenarios":
        from roborun.scenario import list_results
        rows = list_results(limit=int(args.get("limit", 15)),
                            suite=args.get("suite"), outcome=args.get("outcome"),
                            name=args.get("name"))
        if not rows:
            return "No scenario runs recorded yet."
        lines = []
        for r in rows:
            ev = r.get("evaluation") or {}
            ev_s = (" · " + ", ".join(f"{g}:{v}" for g, v in ev.items())) if ev else ""
            tail = f" — {r['reason']}" if r.get("reason") else ""
            lines.append(f"{r['name']} [{r['outcome']}] "
                         f"suite={r.get('suite') or '-'} "
                         f"{r.get('metrics', {})}{ev_s}{tail}")
        return "\n".join(lines)

    elif name == "list_suites":
        from roborun.scenario import list_suites
        cards = list_suites()
        if not cards:
            return "No suites yet — runs scored with a suite= group here."
        return "\n".join(
            f"{c['suite']}: {int(c['pass_rate']*100)}% pass "
            f"({c['passed']}/{c['runs']} runs) · {', '.join(c['scenarios'][:6])}"
            for c in cards)

    return f"Unknown tool: {name}"


def _resolve_behavior_path(name: str) -> Path | None:
    """Map a behavior name / file stem to its source file."""
    from roborun.behaviors import BehaviorRunner
    stem = name.strip().removesuffix(".py")
    for s in BehaviorRunner.get().statuses():
        if s.get("name") == name or Path(s.get("file", "")).stem == stem:
            return Path(s["file"])
    cand = Path("behaviors") / f"{stem}.py"
    return cand if cand.exists() else None


def _read_behavior_source(name: str) -> str:
    path = _resolve_behavior_path(name)
    if path is None:
        return f"No behavior file for {name!r}. Use list_behaviors to see what exists."
    try:
        return f"# {path}\n{path.read_text()}"
    except Exception as exc:
        return f"Could not read {path}: {exc}"


def _behavior_health(name: str) -> str:
    """One-line health for a just-written behavior, by file stem."""
    from roborun.behaviors import BehaviorRunner
    stem = name.strip().removesuffix(".py")
    for s in BehaviorRunner.get().statuses():
        if Path(s.get("file", "")).stem == stem:
            if s.get("last_error"):
                return (f"Loaded but erroring: {s['last_error']} "
                        f"({s.get('errors',0)} errors). Read the timeline and fix it.")
            run = "running" if s.get("enabled") else "loaded (stopped)"
            return f"It's {run} — {s.get('runs',0)} ticks, no errors so far."
    return ("Not visible in the runner yet — it may have failed to import. "
            "Check read_timeline for a load error.")


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

    MAX_TOOL_ROUNDS = 12  # authoring loops (write → observe → fix) need headroom

    @property
    def MODEL(self) -> str:
        from roborun import llm
        provider, model = llm.resolve("smart")
        return model if provider == "anthropic" else "claude-opus-4-8"

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
                    from roborun.events import emit
                    emit("agent", "claude", f"tool: {block.name}", dict(block.input or {}))
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


def _call_local_api(path: str, payload: dict | None = None, method: str = "POST") -> dict:
    port = int(os.environ.get("ROBORUN_PORT", "8765"))
    url = f"http://127.0.0.1:{port}{path}"
    body = json.dumps(payload or {}).encode() if payload is not None else b"{}"
    req = urllib.request.Request(url, data=body,
                                  headers={"Content-Type": "application/json"}, method=method)
    try:
        with urllib.request.urlopen(req, timeout=15) as resp:
            return json.loads(resp.read().decode())
    except Exception as exc:
        return {"ok": False, "error": str(exc)}

