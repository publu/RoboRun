"""Vibecode runtime — drop a Python file in behaviors/, save, robot changes live.

A behavior is a decorated function that gets called in a loop with a `robot`
handle. Files are hot-reloaded on save while the robot is running — no
restart, no build step, no framework to learn:

    # behaviors/follow_person.py
    from roborun.behaviors import behavior

    @behavior(hz=10)
    def follow_person(robot):
        people = robot.see("person")
        if not people:
            return robot.stop()
        robot.move(forward=0.3, turn=-1.2 * (people[0].cx - 0.5))

The `robot` handle:
    robot.see(label=None)    detections (normalized .cx .cy .w .h .label .conf)
    robot.move(forward=0, strafe=0, turn=0)   sim or real robot, safety-clamped
    robot.stop()
    robot.say(text)          speaks into the event timeline
    robot.ask(prompt, image=False, model="fast")   LLM, sync — every= loops only
    robot.think(prompt) / robot.thought()    async LLM — safe at 10 Hz
    robot.delegate(task)     async LLM with tools — may rewrite this policy live
    robot.tool(name, **args) call any MCP tool
    robot.lidar()            360° ranges (m), [0] = straight ahead
    robot.remember(k, v) / robot.recall(k)    persistent key-value memory
    robot.log(msg)           write to the black box
    robot.state              dict that survives across loop ticks (not reloads)

Every move/say/log lands in the tamper-evident event timeline.
"""
from __future__ import annotations

import importlib.util
import json
import os
import threading
import time
import traceback
from pathlib import Path
from typing import Any, Callable

from roborun.events import emit

MAX_LINEAR = float(os.environ.get("ROBORUN_MAX_LINEAR_VEL", "1.0"))
MAX_ANGULAR = float(os.environ.get("ROBORUN_MAX_ANGULAR_VEL", "1.5"))
_MEMORY_PATH = Path(".roborun") / "behavior_memory.json"

# async thought store: (behavior_name, key) -> {"pending": bool, "answer": str|None}
_thoughts: dict[tuple[str, str], dict] = {}
_thoughts_lock = threading.Lock()


def behavior(hz: float | None = None, every: float | None = None,
             name: str | None = None, autostart: bool = True) -> Callable:
    """Mark a function as a behavior loop. `hz` for control loops,
    `every` (seconds) for slow loops. Defaults to 2 Hz."""
    period = (1.0 / hz) if hz else (every if every else 0.5)

    def mark(fn: Callable) -> Callable:
        fn._behavior = {"period": max(0.02, period),
                        "name": name or fn.__name__,
                        "autostart": autostart}
        return fn
    return mark


class Thing:
    """A detection, normalized to [0,1] coordinates. `dist` is meters when
    the source knows it (arena ground truth), else None (webcam)."""
    __slots__ = ("label", "conf", "track_id", "cx", "cy", "w", "h", "dist")

    def __init__(self, det: dict, fw: float, fh: float) -> None:
        x1, y1, x2, y2 = det["bbox"]
        self.label = det["label"]
        self.conf = det["confidence"]
        self.track_id = det.get("track_id", -1)
        self.dist = det.get("distance")
        self.cx = ((x1 + x2) / 2) / fw
        self.cy = ((y1 + y2) / 2) / fh
        self.w = (x2 - x1) / fw
        self.h = (y2 - y1) / fh

    def __repr__(self) -> str:
        return f"<{self.label} {self.conf:.0%} at ({self.cx:.2f},{self.cy:.2f})>"


class Robot:
    """The handle a behavior receives. One per behavior loop."""

    def __init__(self, behavior_name: str) -> None:
        self._name = behavior_name
        self._last_cmd: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._last_move_emit = 0.0
        self._warned_no_actuator = False
        self._last_say = ""
        self.state: dict[str, Any] = {}

    # ---- perception ----

    def see(self, label: str | None = None, min_conf: float = 0.4) -> list[Thing]:
        try:
            from roborun.arena import get_arena
            arena = get_arena()
            if arena.is_active():
                # The arena publishes ground-truth detections on a virtual
                # 1280x720 frame — same shape as the webcam pipeline.
                things = [Thing(d, 1280, 720) for d in arena.detections()
                          if d.get("confidence", 1.0) >= min_conf]
            else:
                from roborun.ros_camera import get_ros_camera
                cam = get_ros_camera()
                if cam.is_active():
                    # connected robot: its camera, its YOLO detections
                    frame = cam.snapshot()
                    fh, fw = frame.shape[:2] if frame is not None else (720, 1280)
                    things = [Thing(d, fw, fh) for d in cam.get_detections()
                              if d["confidence"] >= min_conf]
                else:
                    from roborun.routes._singletons import get_webcam
                    wc = get_webcam()
                    frame = wc.snapshot()
                    fh, fw = frame.shape[:2] if frame is not None else (720, 1280)
                    things = [Thing(d, fw, fh) for d in wc.get_detections()
                              if d["confidence"] >= min_conf]
        except Exception:
            return []
        if label:
            things = [t for t in things if t.label == label]
        return sorted(things, key=lambda t: -t.conf)

    # ---- async LLM: the policy never stops driving while it thinks ----

    def think(self, prompt: str, key: str = "default", image: bool = False,
              system: str | None = None, max_tokens: int = 300,
              model: str = "smart") -> bool:
        """Fire-and-forget ask — safe inside `hz=` loops. Returns True if a
        new thought started, False if one with this key is still pending
        (re-calls are no-ops, so calling every tick is fine). Collect the
        answer later with robot.thought(key)."""
        k = (self._name, key)
        with _thoughts_lock:
            slot = _thoughts.get(k)
            if slot is not None and slot["pending"]:
                return False
            _thoughts[k] = {"pending": True, "answer": None}
        jpeg = self.frame_jpeg() if image else None

        def _run() -> None:
            from roborun import llm
            try:
                ans = llm.complete(prompt, system=system, image_jpeg=jpeg,
                                   tier=model, max_tokens=max_tokens)
            except Exception as exc:
                ans = f"[think failed: {exc}]"
            with _thoughts_lock:
                _thoughts[k] = {"pending": False, "answer": ans}
            emit("agent", self._name, f"thought ready · {key}", {"key": key})

        threading.Thread(target=_run, daemon=True, name=f"think-{key}").start()
        return True

    def thought(self, key: str = "default") -> str | None:
        """Pop a completed thought, or None while pending/absent."""
        with _thoughts_lock:
            slot = _thoughts.get((self._name, key))
            if slot is not None and not slot["pending"]:
                del _thoughts[(self._name, key)]
                return slot["answer"]
        return None

    def thinking(self, key: str = "default") -> bool:
        with _thoughts_lock:
            slot = _thoughts.get((self._name, key))
            return slot is not None and slot["pending"]

    def delegate(self, task: str, key: str = "delegate", max_steps: int = 4,
                 model: str = "smart") -> bool:
        """Hand a task to the LLM *with hands*: it may call any MCP tool —
        move, navigate, camera_snapshot, write_behavior (yes, it can
        rewrite this very policy; hot reload applies it live). Async like
        think(); every action lands in the timeline; the final report
        arrives via robot.thought(key). Actuation still passes through the
        same clamps and estop as everything else."""
        k = (self._name, key)
        with _thoughts_lock:
            slot = _thoughts.get(k)
            if slot is not None and slot["pending"]:
                return False
            _thoughts[k] = {"pending": True, "answer": None}

        def _run() -> None:
            from roborun import llm
            from roborun.ros_mcp import get_all_tools, handle_tool_call
            catalog = "\n".join(f"- {t['name']}: {t['description'][:140]}"
                                 for t in get_all_tools())
            system = (
                "You are a robot's delegated executor. Reply with ONLY one JSON "
                "object per turn, no prose, no code fences:\n"
                '  {"tool": "<name>", "args": {...}}   to act, or\n'
                '  {"done": "<short report>"}          when finished.\n'
                f"Available tools:\n{catalog}")
            transcript = f"Task from the running policy '{self._name}': {task}"
            report = "[delegate: no result]"
            try:
                for _ in range(max_steps):
                    raw = llm.complete(transcript, system=system,
                                       tier=model, max_tokens=500)
                    action = _parse_action(raw)
                    if action is None or "done" in action:
                        report = (action or {}).get("done", raw.strip())
                        break
                    name = str(action.get("tool", ""))
                    args = action.get("args") or {}
                    result = handle_tool_call(name, args)
                    emit("agent", self._name,
                         f"delegate · {name}({json.dumps(args)[:80]}) → "
                         f"{'ok' if result.get('ok') else result.get('error', 'failed')}",
                         {"tool": name})
                    transcript += (f"\n\nYou called {name} with {json.dumps(args)}."
                                   f"\nResult: {json.dumps(result, default=str)[:1200]}"
                                   f"\nNext JSON:")
                else:
                    report = f"[delegate: stopped after {max_steps} steps]"
            except Exception as exc:
                report = f"[delegate failed: {exc}]"
            with _thoughts_lock:
                _thoughts[k] = {"pending": False, "answer": report}
            emit("agent", self._name, f"delegate done · {report[:120]}", {"key": key})

        threading.Thread(target=_run, daemon=True, name=f"delegate-{key}").start()
        return True

    def lidar(self) -> list[float]:
        """360° range scan in meters, index 0 = straight ahead, CCW.
        Arena provides it; webcam mode has no lidar -> []."""
        try:
            from roborun.arena import get_arena
            a = get_arena()
            return a.lidar() if a.is_active() else []
        except Exception:
            return []

    def tool(self, name: str, **args: Any) -> dict:
        """Call any MCP tool from inside a behavior — same registry your
        Claude uses (camera_snapshot, navigate, skill tools, ...). Slow
        tools belong in `every=` loops, same rule as robot.ask()."""
        try:
            from roborun.ros_mcp import handle_tool_call
            return handle_tool_call(name, args)
        except Exception as exc:
            return {"ok": False, "error": str(exc)}

    def frame_jpeg(self) -> bytes | None:
        p = Path("/tmp/roborun_frame.jpg")
        try:
            return p.read_bytes() if p.exists() else None
        except Exception:
            return None

    # ---- action ----

    def move(self, forward: float = 0.0, strafe: float = 0.0, turn: float = 0.0) -> None:
        forward = max(-MAX_LINEAR, min(MAX_LINEAR, forward))
        strafe = max(-MAX_LINEAR, min(MAX_LINEAR, strafe))
        turn = max(-MAX_ANGULAR, min(MAX_ANGULAR, turn))

        sent = False
        try:
            from roborun.arena import get_arena
            arena = get_arena()
            if arena.is_active():
                arena.set_cmd(forward, strafe, turn)
                sent = True
        except Exception:
            pass
        if not sent:
            try:
                from roborun.routes._singletons import get_simulator
                sim = get_simulator()
                if sim.is_running:
                    sim.set_cmd_vel(forward, strafe, turn)
                    sent = True
            except Exception:
                pass
        if not sent:
            try:
                from roborun.rosbridge import get_client
                client = get_client(auto_connect=False)
                if client and client.health.get("connected"):
                    client.move(forward, strafe, turn)
                    sent = True
            except Exception:
                pass

        if not sent:
            # webcam-only mode: say it once, then stay quiet until an actuator shows up
            if not self._warned_no_actuator:
                self._warned_no_actuator = True
                emit("system", self._name,
                     "wants to move, but no actuator — start the sim or connect "
                     "a robot (further move logs muted)", {})
            return
        self._warned_no_actuator = False

        # Real actuator: log on sharp changes immediately, otherwise ≤1/sec.
        cmd = (forward, strafe, turn)
        delta = max(abs(a - b) for a, b in zip(cmd, self._last_cmd))
        now = time.monotonic()
        if delta >= 0.3 or (delta > 0.01 and now - self._last_move_emit >= 1.0):
            self._last_cmd = cmd
            self._last_move_emit = now
            emit("ros", self._name, f"move fwd={forward:.2f} turn={turn:.2f}",
                 {"forward": round(forward, 2), "strafe": round(strafe, 2),
                  "turn": round(turn, 2)})

    def stop(self) -> None:
        self.move(0.0, 0.0, 0.0)

    def say(self, text: str) -> None:
        text = str(text).strip()
        if text and text != self._last_say:
            self._last_say = text
            emit("agent", self._name, text, {})

    def log(self, msg: str, **detail: Any) -> None:
        emit("task", self._name, str(msg), detail)

    # ---- memory ----

    def remember(self, key: str, value: Any) -> None:
        data = self._load_memory()
        data[key] = value
        _MEMORY_PATH.parent.mkdir(parents=True, exist_ok=True)
        _MEMORY_PATH.write_text(json.dumps(data, default=str, indent=1))

    def recall(self, key: str, default: Any = None) -> Any:
        return self._load_memory().get(key, default)

    @staticmethod
    def _load_memory() -> dict:
        try:
            return json.loads(_MEMORY_PATH.read_text())
        except Exception:
            return {}

    # ---- LLM (provider-agnostic, tiered — see roborun/llm.py) ----

    def ask(self, prompt: str, image: bool = False, system: str | None = None,
            max_tokens: int = 300, model: str = "fast") -> str:
        """Ask an LLM. `model` is a tier ("fast" for frequent cheap calls,
        "smart" for reasoning) or an explicit "provider:model" spec —
        anthropic, openai, gemini, ollama, or any OpenAI-compatible endpoint
        via OPENAI_BASE_URL. `image=True` attaches the current camera frame.

        Never call this from a `hz=` loop — it belongs in `every=` loops
        and tools (docs/SPEED_LAYERS.md, contract 1)."""
        from roborun import llm
        try:
            return llm.complete(prompt, system=system,
                                image_jpeg=self.frame_jpeg() if image else None,
                                tier=model, max_tokens=max_tokens)
        except Exception as exc:
            emit("system", self._name, f"ask() failed: {exc}", {})
            return f"[ask failed: {exc}]"


class _Loop:
    """One running behavior: its thread, its robot handle, its stats."""

    def __init__(self, name: str, fn: Callable, period: float, source: str) -> None:
        self.name = name
        self.fn = fn
        self.period = period
        self.source = source
        self.robot = Robot(name)
        self.enabled = True
        self.runs = 0
        self.errors = 0
        self.last_error: str | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name=f"behavior:{name}")

    def start(self) -> None:
        self._thread.start()

    def halt(self) -> None:
        self._stop.set()

    def _run(self) -> None:
        while not self._stop.is_set():
            if not self.enabled:
                time.sleep(0.2)
                continue
            t0 = time.monotonic()
            try:
                self.fn(self.robot)
                self.runs += 1
            except Exception as exc:
                self.errors += 1
                self.last_error = f"{type(exc).__name__}: {exc}"
                line = traceback.format_exc().strip().splitlines()[-1]
                emit("system", self.name, f"behavior error: {line}", {})
                time.sleep(2.0)  # don't spin on a broken loop
            elapsed = time.monotonic() - t0
            self._stop.wait(max(0.0, self.period - elapsed))
        try:
            self.robot.stop()
        except Exception:
            pass

    def status(self) -> dict[str, Any]:
        return {"name": self.name, "file": self.source, "enabled": self.enabled,
                "period": self.period, "runs": self.runs, "errors": self.errors,
                "last_error": self.last_error}


class BehaviorRunner:
    """Watches behaviors/ directories, hot-reloads files on save."""

    _instance: "BehaviorRunner | None" = None

    def __init__(self) -> None:
        self.dirs = [Path("behaviors")]
        for extra in os.environ.get("ROBORUN_BEHAVIOR_PATHS", "").split(","):
            if extra.strip():
                self.dirs.append(Path(extra.strip()).expanduser())
        self._mtimes: dict[Path, float] = {}
        self._loops: dict[Path, list[_Loop]] = {}
        self._watcher: threading.Thread | None = None
        self._stop = threading.Event()

    @classmethod
    def get(cls) -> "BehaviorRunner":
        if cls._instance is None:
            cls._instance = BehaviorRunner()
        return cls._instance

    def start(self) -> None:
        if self._watcher:
            return
        self._watcher = threading.Thread(target=self._watch, daemon=True,
                                         name="BehaviorWatcher")
        self._watcher.start()

    def stop(self) -> None:
        self._stop.set()
        for loops in self._loops.values():
            for loop in loops:
                loop.halt()

    def statuses(self) -> list[dict]:
        out = []
        for loops in self._loops.values():
            out.extend(loop.status() for loop in loops)
        return sorted(out, key=lambda s: s["name"])

    def set_enabled(self, name: str, enabled: bool) -> bool:
        for loops in self._loops.values():
            for loop in loops:
                if loop.name == name:
                    loop.enabled = enabled
                    if not enabled:
                        loop.robot.stop()
                    emit("system", "behaviors",
                         f"{name} {'enabled' if enabled else 'disabled'}", {})
                    return True
        return False

    def _watch(self) -> None:
        while not self._stop.is_set():
            try:
                self._scan()
            except Exception:
                pass
            self._stop.wait(1.0)

    def _scan(self) -> None:
        seen: set[Path] = set()
        for d in self.dirs:
            if not d.is_dir():
                continue
            for path in sorted(d.glob("*.py")):
                if path.name.startswith("_"):
                    continue
                seen.add(path)
                mtime = path.stat().st_mtime
                if self._mtimes.get(path) != mtime:
                    self._mtimes[path] = mtime
                    self._load(path, reload=path in self._loops)
        # files deleted → halt their loops
        for path in [p for p in self._loops if p not in seen]:
            for loop in self._loops.pop(path):
                loop.halt()
            self._mtimes.pop(path, None)
            emit("system", "behaviors", f"unloaded {path.name}", {})

    def _load(self, path: Path, reload: bool = False) -> None:
        for loop in self._loops.pop(path, []):
            loop.halt()
        try:
            spec = importlib.util.spec_from_file_location(
                f"roborun_behavior_{path.stem}_{int(time.time() * 1000)}", path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
        except Exception as exc:
            line = traceback.format_exc().strip().splitlines()[-1]
            emit("system", "behaviors", f"{path.name} failed to load: {line}", {})
            return

        loops: list[_Loop] = []
        for attr in vars(module).values():
            meta = getattr(attr, "_behavior", None)
            if meta and callable(attr):
                loop = _Loop(meta["name"], attr, meta["period"], str(path))
                loop.enabled = meta["autostart"]
                loop.start()
                loops.append(loop)
        if loops:
            self._loops[path] = loops
            names = ", ".join(l.name for l in loops)
            emit("system", "behaviors",
                 f"{'reloaded' if reload else 'loaded'} {path.name} → {names}", {})


# ---- starter behaviors written on first boot ----

EXAMPLES: dict[str, str] = {
    "follow_person.py": '''\
"""Follow whoever the camera sees. Edit the numbers, save, watch it change."""
from roborun.behaviors import behavior


@behavior(hz=10)
def follow_person(robot):
    people = robot.see("person")
    if not people:
        return robot.stop()
    target = people[0]
    robot.move(
        forward=0.3 if target.h < 0.6 else 0.0,   # stop when close
        turn=-1.2 * (target.cx - 0.5),            # steer toward center
    )
''',
    "patrol.py": '''\
"""Square patrol in the sim. `roborun` autostarts MuJoCo if there's no camera."""
import time
from roborun.behaviors import behavior


@behavior(hz=5, autostart=False)
def patrol(robot):
    leg = robot.state.setdefault("leg", {"mode": "walk", "until": time.time() + 3})
    if time.time() > leg["until"]:
        leg["mode"] = "turn" if leg["mode"] == "walk" else "walk"
        leg["until"] = time.time() + (1.6 if leg["mode"] == "turn" else 3)
        robot.log(f"patrol: {leg['mode']}")
    robot.move(forward=0.4 if leg["mode"] == "walk" else 0.0,
               turn=0.9 if leg["mode"] == "turn" else 0.0)
''',
    "explore.py": '''\
"""Chamber explorer: forward until something blocks, then turn. Good enough
to clear Arena chamber 01 — open http://localhost:8765/arena and enable me.
Sighted doors and explored rooms land in the timeline as you go."""
from roborun.behaviors import behavior


@behavior(hz=10, autostart=False)
def explore(robot):
    blocked = [t for t in robot.see("obstacle") if t.h > 0.45]
    if blocked:
        robot.move(turn=1.1)          # too close to a wall: rotate
    else:
        robot.move(forward=0.8, turn=0.15)   # gentle arc covers rooms
''',
    "heartbeat.py": '''\
"""User-defined supervisor: write a HEARTBEAT.md and this runs it on your
schedule against live system status. No file, no LLM calls — this loop
idles for free until you create one.

HEARTBEAT.md is your prompt: what to watch, what to flag, what to propose.
Optional first line `every: 600` sets the interval in seconds (default 600).

    every: 900
    You supervise this robot. Flag anything odd in the behavior statuses,
    check whether a run is being recorded, and propose one improvement.
"""
import json
import time
from pathlib import Path

from roborun.behaviors import behavior

_PATHS = (Path("HEARTBEAT.md"), Path.home() / ".roborun" / "HEARTBEAT.md")


def _status_snapshot():
    from roborun.behaviors import BehaviorRunner
    from roborun.recorder import active_recorder
    rec = active_recorder()
    return {
        "behaviors": BehaviorRunner.get().statuses(),
        "recording": rec.status() if rec else None,
        "time": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    }


@behavior(every=30.0)
def heartbeat(robot):
    path = next((p for p in _PATHS if p.exists()), None)
    if path is None:
        return  # no HEARTBEAT.md — do nothing, cost nothing
    text = path.read_text().strip()
    interval = 600.0
    if text.startswith("every:"):
        first, _, text = text.partition("\\n")
        interval = float(first.split(":", 1)[1].strip() or 600)
    if time.time() - robot.recall("heartbeat_last", 0) < interval:
        return
    robot.remember("heartbeat_last", time.time())
    answer = robot.ask(
        f"{text}\\n\\nCurrent system status:\\n"
        f"{json.dumps(_status_snapshot(), default=str, indent=1)}",
        model="smart", max_tokens=500)
    robot.log("heartbeat", report=answer)
    robot.say(answer)
''',
}


def _parse_action(raw: str) -> dict | None:
    """First JSON object in an LLM reply; tolerates code fences."""
    text = raw.strip()
    if text.startswith("```"):
        text = text.strip("`")
        if text.startswith("json"):
            text = text[4:]
    start = text.find("{")
    if start < 0:
        return None
    depth = 0
    for i in range(start, len(text)):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                try:
                    return json.loads(text[start:i + 1])
                except Exception:
                    return None
    return None


def write_behavior_file(name: str, source: str) -> dict:
    """Validated write into behaviors/ — shared by the MCP tool and the
    arena editor. Never executes the source; hot reload does that."""
    import re
    name = name.strip().removesuffix(".py")
    if not re.match(r"^[a-z0-9_]+$", name):
        return {"ok": False, "error": "name must be a snake_case slug"}
    if "def " not in source or "@behavior" not in source:
        return {"ok": False, "error": "source must define an @behavior-decorated "
                                      "function (from roborun.behaviors import behavior)"}
    try:
        compile(source, f"{name}.py", "exec")
    except SyntaxError as exc:
        return {"ok": False, "error": f"syntax error: {exc}"}
    path = Path("behaviors") / f"{name}.py"
    path.parent.mkdir(exist_ok=True)
    path.write_text(source)
    return {"ok": True, "path": str(path),
            "note": "hot reload picks it up within ~1s"}


def write_examples(target: Path | None = None) -> Path | None:
    """Create behaviors/ with starter files if it doesn't exist."""
    target = target or Path("behaviors")
    if target.exists():
        return None
    target.mkdir(parents=True)
    for name, source in EXAMPLES.items():
        (target / name).write_text(source)
    return target
