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
    robot.pose()             {x, z, heading} odometry (arena / robots with odom)
    robot.seen(label=None)   the system's automatic sighting memory this run
    robot.goto(x, z)         one tick of drive-toward; True when arrived
    robot.clearance()        {"ahead","left","right","behind"} wall distances
    robot.openings()         doorways found in the mapped walls, nearest first
    robot.frontier()         (x,z) edge of unseen space — None when fully seen
    robot.route(x, z)        next waypoint through space known to be clear
    robot.mapped()           cells of spatial memory so far
    robot.locate(thing)      project a sighting to world (x, z)
    robot.approach(thing)    locate + goto in one verb
    robot.move(climb=)       vertical for drones (Twist linear.z)
    robot.grasp(closed)      gripper, when the connected robot has one
    robot.answer("6")        submit the chamber's answer (recon levels)
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

    def seen(self, label: str | None = None) -> list[dict]:
        """What the system has observed this run — automatic, no
        bookkeeping. Episode-counted sightings with sample poses:
        robot.seen("red door") -> [{"label", "count", "poses", ...}].
        This is the answer source for recon questions; the same
        observations are already in the timeline and the sealed run."""
        try:
            from roborun.sightings import summary
            return summary(label)
        except Exception:
            return []

    def pose(self) -> dict | None:
        """{x, z, heading} in the handle frame — from the arena when it's
        live, otherwise from the connected robot's /odom (ros_telemetry
        keeps it handle-shaped). None means stale or absent odometry:
        go blind loudly, never act on the past."""
        try:
            from roborun.arena import get_arena
            a = get_arena()
            if a.is_active():
                return a.pose()
        except Exception:
            pass
        try:
            from roborun.ros_telemetry import get_bridge
            return get_bridge().handle_pose()
        except Exception:
            return None

    FOV = 1.323   # see(): bearing_rad = (0.5 - cx) * FOV

    def locate(self, thing) -> tuple[float, float] | None:
        """Project a see() sighting into world coordinates (x, z).
        Needs pose and the thing's dist; None if either is missing."""
        import math
        pose = self.pose()
        if pose is None or getattr(thing, "dist", None) is None:
            return None
        a = pose["heading"] + (0.5 - thing.cx) * self.FOV
        return (pose["x"] + math.cos(a) * thing.dist,
                pose["z"] - math.sin(a) * thing.dist)

    def approach(self, thing, tol: float = 0.45) -> bool:
        """Drive toward a sighting: locate() + goto() in one verb.
        True when arrived (or unlocatable -> False, stopped)."""
        t = self.locate(thing)
        if t is None:
            self.stop()
            return False
        label = getattr(thing, "label", "target")
        dist = getattr(thing, "dist", None)
        self._intent("approach", f"{label}" + (f" · {dist:.1f}m" if dist else ""), t)
        self._in_verb = True
        try:
            return self.goto(t[0], t[1], tol=tol)
        finally:
            self._in_verb = False

    def _intent(self, verb: str, detail: str,
                target: tuple[float, float] | None = None) -> None:
        """Report what this verb is trying to do — the arena narrates it
        live (status line, map marker). Cheap, fire-and-forget."""
        try:
            from roborun.arena import get_arena
            a = get_arena()
            if a.is_active():
                a.set_intent({"verb": verb, "detail": detail,
                              "target": list(target) if target else None})
        except Exception:
            pass

    def _spatial(self):
        """Fold the current scan into spatial memory (the occupancy grid
        behind mapped/frontier/route). Returns (grid, pose) or (None, None)."""
        pose, scan = self.pose(), self.lidar()
        if not pose or not scan:
            return None, None
        ex = self.state.setdefault("_spatial", {"grid": {}, "tick": 0})
        ex["tick"] += 1
        _explore_integrate(ex["grid"], pose, scan)
        return ex["grid"], pose

    def mapped(self) -> int:
        """How much world the robot has seen: cells of spatial memory."""
        grid, _ = self._spatial()
        return len(grid) if grid else 0

    def frontier(self, prefer: str = "near") -> tuple[float, float] | None:
        """WHERE IS NEW SPACE? The edge between what the robot has seen
        and what it hasn't, as a world point — or None once nothing
        reachable is unexplored (that's your "done").

        prefer="near": closest unseen edge — systematic sweep.
        prefer="far":  deepest unseen edge — push into the unknown first.

        The explore loop is yours to write:

            t = robot.frontier()
            if t is None: ...done...
            robot.goto(*(robot.route(*t) or t))
        """
        grid, pose = self._spatial()
        if grid is None:
            return None
        # hysteresis: hold the chosen frontier until it's been seen (the
        # map swallowed it) or it ages out — a target that flip-flops
        # every tick as the map grows would thrash the robot
        fr = self.state.setdefault("_frontier", {"cell": None, "age": 0,
                                                 "prefer": prefer})
        fr["age"] += 1
        c = fr["cell"]
        if (c is not None and fr["prefer"] == prefer and fr["age"] < 20
                and grid.get(c) is None):
            return (c[0] * _EXPLORE_CELL, c[1] * _EXPLORE_CELL)
        fronts = _explore_frontiers(grid, pose)
        if not fronts:
            fr["cell"] = None
            return None
        c = fronts[0] if prefer == "near" else fronts[-1]
        fr.update(cell=c, age=0, prefer=prefer)
        return (c[0] * _EXPLORE_CELL, c[1] * _EXPLORE_CELL)

    def clearance(self) -> dict:
        """WALLS, in plain words: nearest obstacle distance around the
        robot — {"ahead", "left", "right", "behind"} in meters. Just the
        lidar, sliced the way you'd ask the question:

            if robot.clearance()["ahead"] < 0.6:  # wall coming up
        """
        scan = self.lidar()
        if not scan:
            return {"ahead": None, "left": None, "right": None, "behind": None}
        n = len(scan)
        q = n // 4
        sector = lambda i: min(scan[(i - 2) % n], scan[(i - 1) % n], scan[i % n],
                               scan[(i + 1) % n], scan[(i + 2) % n])
        return {"ahead": sector(0), "left": sector(q),
                "behind": sector(2 * q), "right": sector(3 * q)}

    def openings(self) -> list[tuple[float, float]]:
        """DOORS, structurally: gaps in the walls the robot has mapped,
        wide enough to drive through — doorways, gates, passages — as
        world points, nearest first. Derived from spatial memory, so the
        robot has to have seen the wall to know about the hole in it.

            for door in robot.openings():
                if robot.goto(*door): ...
        """
        import math
        grid, pose = self._spatial()
        if grid is None:
            return []
        cells = []
        for (cx, cz), v in list(grid.items()):
            if v != 1:
                continue
            # a doorway cell: walls close on both sides of one axis,
            # open passage along the other
            for (wa, wb), (pa, pb) in ((((1, 0), (-1, 0)), ((0, 1), (0, -1))),
                                       (((0, 1), (0, -1)), ((1, 0), (-1, 0)))):
                if (_near_wall(grid, cx, cz, wa) and _near_wall(grid, cx, cz, wb)
                        and _open_run(grid, cx, cz, pa) and _open_run(grid, cx, cz, pb)):
                    cells.append((cx, cz))
                    break
        # cluster adjacent doorway cells into one opening each
        out = []
        used = set()
        for c in cells:
            if c in used:
                continue
            group = [c]
            used.add(c)
            queue = [c]
            while queue:
                gx, gz = queue.pop()
                for o in cells:
                    if o not in used and abs(o[0] - gx) <= 2 and abs(o[1] - gz) <= 2:
                        used.add(o)
                        group.append(o)
                        queue.append(o)
            out.append((sum(g[0] for g in group) / len(group) * _EXPLORE_CELL,
                        sum(g[1] for g in group) / len(group) * _EXPLORE_CELL))
        out.sort(key=lambda w: math.hypot(w[0] - pose["x"], w[1] - pose["z"]))
        return out

    def route(self, x: float, z: float) -> tuple[float, float] | None:
        """HOW DO I GET THERE THROUGH WHAT I KNOW? Next waypoint toward
        (x, z) routed through cells the robot has seen to be clear,
        walls inflated. None = no known-clear path (yet) — drive
        somewhere new or fall back to goto. Replans as the map grows."""
        import math
        grid, pose = self._spatial()
        if grid is None:
            return None
        cell = (round(x / _EXPLORE_CELL), round(z / _EXPLORE_CELL))
        rt = self.state.setdefault("_route", {"target": None, "path": None, "age": 0})
        rt["age"] += 1
        if rt["target"] != cell or rt["age"] >= 10 or not rt["path"]:
            rt["target"], rt["age"] = cell, 0
            rt["path"] = _explore_route(grid, pose, cell)
        path = rt["path"]
        if not path:
            return None
        while path and math.hypot(path[0][0] * _EXPLORE_CELL - pose["x"],
                                  path[0][1] * _EXPLORE_CELL - pose["z"]) < 0.45:
            path.pop(0)
        if not path:
            return (x, z)                       # already on top of it
        c = path[min(2, len(path) - 1)]      # near waypoint: stay in the
        return (c[0] * _EXPLORE_CELL, c[1] * _EXPLORE_CELL)  # planned channel

    def goto(self, x: float, z: float, speed: float = 0.9,
             tol: float = 0.45) -> bool:
        """One tick of drive-toward-point. Call it every tick; it steers
        and returns True once within tol. Needs pose (arena / odom
        robots). Steering owns the last meter — including not parking
        its nose on a wall: when lidar shows < 0.55 m dead ahead it
        commits to one relief turn until clear (commitment, not
        flip-flop). Choosing WHERE to go stays your policy's job."""
        if not getattr(self, "_in_verb", False):
            self._intent("goto", f"→ ({x:.1f}, {z:.1f})", (x, z))
        import math
        pose = self.pose()
        if pose is None:
            self.stop()
            return False
        dx, dz = x - pose["x"], z - pose["z"]
        if math.hypot(dx, dz) < tol:
            self.stop()
            return True
        scan = self.lidar()
        if scan:
            ahead = min(scan[:2] + scan[-2:])
            relief = self.state.get("_goto_relief")
            if relief is not None:
                # committed: keep turning the same way until actually
                # clear — half-measures here are how robots livelock
                if ahead > 0.9:
                    self.state["_goto_relief"] = None
                else:
                    self.move(forward=0.15, turn=relief * 1.2)
                    return False
            elif ahead < 0.55:
                left, right = sum(scan[6:12]), sum(scan[-12:-6])
                self.state["_goto_relief"] = 1.0 if left > right else -1.0
                self.move(forward=0.15, turn=self.state["_goto_relief"] * 1.2)
                return False
        bearing = (math.atan2(-dz, dx) - pose["heading"] + math.pi) \
            % (2 * math.pi) - math.pi
        if abs(bearing) > 0.5:
            self.move(turn=1.2 if bearing > 0 else -1.2)
        else:
            self.move(forward=speed, turn=0.8 * bearing)
        return False

    def lidar(self) -> list[float]:
        """360° range scan in meters, 36 sectors, index 0 = straight
        ahead, CCW — from the arena when it's live, otherwise the
        connected robot's /scan resampled to the same schema. Webcam
        mode has no lidar -> []."""
        try:
            from roborun.arena import get_arena
            a = get_arena()
            if a.is_active():
                return a.lidar()
        except Exception:
            pass
        try:
            from roborun.ros_telemetry import get_bridge
            return get_bridge().handle_lidar()
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

    def move(self, forward: float = 0.0, strafe: float = 0.0, turn: float = 0.0,
             climb: float = 0.0) -> None:
        forward = max(-MAX_LINEAR, min(MAX_LINEAR, forward))
        strafe = max(-MAX_LINEAR, min(MAX_LINEAR, strafe))
        turn = max(-MAX_ANGULAR, min(MAX_ANGULAR, turn))
        climb = max(-MAX_LINEAR, min(MAX_LINEAR, climb))  # Twist linear.z

        sent = False
        try:
            from roborun.arena import get_arena
            arena = get_arena()
            if arena.is_active():
                arena.set_cmd(forward, strafe, turn, climb)
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
                    # the robot's own command topic (mavros setpoint for
                    # drones), and climb rides Twist linear.z — the same
                    # four axes the arena takes (SIM_SPEC contract)
                    from roborun.ros_telemetry import get_bridge
                    client.move(forward, strafe, turn,
                                get_bridge().cmd_vel_topic, linear_z=climb)
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

    def grasp(self, closed: bool = True) -> None:
        """Close or open the end-effector, when the robot has a gripper."""
        try:
            from roborun.arena import get_arena
            arena = get_arena()
            if arena.is_active():
                arena.set_grip(closed)
        except Exception:
            pass

    def answer(self, text: str) -> None:
        """Submit an answer to the chamber's question (recon levels).
        Also lands in the timeline, so the sealed run carries it."""
        try:
            from roborun.arena import get_arena
            arena = get_arena()
            if arena.is_active():
                arena.set_answer(str(text))
        except Exception:
            pass
        emit("task", self._name, f"ANSWER: {text}", {"answer": str(text)})

    def say(self, text: str) -> None:
        text = str(text).strip()
        if text and text != self._last_say:
            self._last_say = text
            emit("agent", self._name, text, {})

    def log(self, msg: str, **detail: Any) -> None:
        emit("task", self._name, str(msg), detail)

    def notify(self, text: str, **detail: Any) -> None:
        """Reach the human. Lands in the timeline like log(), and when the
        OpenClaw bridge is configured (OPENCLAW_HOOKS_URL) it is pushed to
        their chat — phone-buzz territory. Use it for "someone should hear
        about this", not progress chatter; that's what log() is for."""
        text = str(text).strip()
        if text:
            emit("notify", self._name, text, detail)

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
    "sentry.py": '''\
"""Patrol that reaches a human. Person in view -> one notify (60s cooldown);
each completed lap -> a status notify. Notifications always land in the
timeline; with the OpenClaw bridge configured they hit your phone too
(docs/OPENCLAW.md)."""
import time
from roborun.behaviors import behavior


@behavior(hz=5, autostart=False)
def sentry(robot):
    leg = robot.state.setdefault("leg", {"mode": "walk", "until": time.time() + 3,
                                         "turns": 0})
    people = robot.see("person")
    if people and time.time() > robot.state.get("quiet_until", 0):
        robot.state["quiet_until"] = time.time() + 60
        robot.notify(f"sentry: {len(people)} person(s) in view", count=len(people))
    if time.time() > leg["until"]:
        if leg["mode"] == "turn":
            leg["turns"] += 1
            if leg["turns"] % 4 == 0:
                robot.notify(f"sentry: lap {leg['turns'] // 4} complete, all quiet")
        leg["mode"] = "turn" if leg["mode"] == "walk" else "walk"
        leg["until"] = time.time() + (1.6 if leg["mode"] == "turn" else 3)
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


_EXPLORE_CELL = 0.25   # finer than the narrowest doorway, or maps plug them


def _explore_integrate(grid: dict, pose: dict, scan: list) -> None:
    import math
    n = len(scan)
    for i, r in enumerate(scan):
        a = pose["heading"] + i / n * 2 * math.pi
        d = _EXPLORE_CELL * 0.5
        while d < r:
            c = (round((pose["x"] + math.cos(a) * d) / _EXPLORE_CELL),
                 round((pose["z"] - math.sin(a) * d) / _EXPLORE_CELL))
            if grid.get(c) != 2:
                grid[c] = 1
            d += _EXPLORE_CELL * 0.5
        if r < 7.5:
            grid[(round((pose["x"] + math.cos(a) * r) / _EXPLORE_CELL),
                  round((pose["z"] - math.sin(a) * r) / _EXPLORE_CELL))] = 2


def _explore_clear(grid: dict, c: tuple) -> bool:
    # traversable = free and not hugging a wall (robots are fatter than cells)
    if grid.get(c) != 1:
        return False
    for dx in (-1, 0, 1):
        for dz in (-1, 0, 1):
            if grid.get((c[0] + dx, c[1] + dz)) == 2:
                return False
    return True


def _near_wall(grid: dict, cx: int, cz: int, d: tuple, reach: int = 3) -> bool:
    for k in range(1, reach + 1):
        if grid.get((cx + d[0] * k, cz + d[1] * k)) == 2:
            return True
    return False


def _open_run(grid: dict, cx: int, cz: int, d: tuple, run: int = 3) -> bool:
    return all(grid.get((cx + d[0] * k, cz + d[1] * k)) == 1
               for k in range(1, run + 1))


def _explore_frontiers(grid: dict, pose: dict) -> list:
    """All frontier cells (unknown space adjacent to known-clear space),
    in BFS order from the robot: [0] is nearest, [-1] is deepest."""
    start = (round(pose["x"] / _EXPLORE_CELL), round(pose["z"] / _EXPLORE_CELL))
    queue, seen, found = [start], {start}, []
    while queue:
        c = queue.pop(0)
        for dx, dz in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            n = (c[0] + dx, c[1] + dz)
            if n in seen:
                continue
            seen.add(n)
            if grid.get(n) is None:
                found.append(n)
            elif _explore_clear(grid, n):
                queue.append(n)
    return found


def _explore_route(grid: dict, pose: dict, target: tuple) -> list | None:
    """BFS path from the robot to `target` through known-clear cells
    (the target itself may be unknown — a frontier). None = unreachable
    through mapped space."""
    start = (round(pose["x"] / _EXPLORE_CELL), round(pose["z"] / _EXPLORE_CELL))
    if start == target:
        return [target]
    prev, queue, seen = {}, [start], {start}
    while queue:
        c = queue.pop(0)
        for dx, dz in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            n = (c[0] + dx, c[1] + dz)
            if n in seen:
                continue
            if n == target:
                path = [n, c]
                while path[-1] != start:
                    path.append(prev[path[-1]])
                return path[::-1]
            seen.add(n)
            if _explore_clear(grid, n):
                prev[n] = c
                queue.append(n)
    return None


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
