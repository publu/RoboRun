"""Browser runtime — the real roborun modules driven by JS ticks.

Loaded by web/wasm.js inside Pyodide when the arena runs without a local
server (e.g., GitHub Pages). The same behaviors.py, sightings.py, and
recorder.py as a local install run against an in-memory filesystem: a run
sealed in the browser carries the identical chunk hash chain + Merkle
seal, and `python -m roborun.recorder verify` accepts the downloaded
files. No anchoring in the browser (no TSA reachable from a static page)
— seals are `consistent_unanchored` by design.

WASM has no threads, so the two thread users are repointed here: the
recorder's event-bus tap subscribes a queue that tick() drains, and
Robot.think() degrades to a logged no-op.
"""
from __future__ import annotations

import json
import queue
import traceback

from roborun import events as bus
from roborun import recorder as rec_mod
from roborun import sightings
from roborun.arena import get_arena
from roborun.behaviors import Robot


def _attach_event_bus_sync(self) -> None:
    if self._bus_queue is None:
        self._bus_queue = bus.subscribe()


rec_mod.RunRecorder.attach_event_bus = _attach_event_bus_sync


def _think_browser(self, prompt, key="default", **kw) -> bool:
    if not self.state.get("_think_warned"):
        self.state["_think_warned"] = True
        self.log("robot.think() needs the local install — pip install ros-agent")
    return False


Robot.think = _think_browser

_policy = None
_robot: Robot | None = None
_policy_error: str | None = None
_last_level = ""


def load_policy(source: str) -> str:
    """Validate + load a policy, same rules as write_behavior_file."""
    global _policy, _robot, _policy_error
    if "def " not in source or "@behavior" not in source:
        return json.dumps({"ok": False, "error": "source must define an "
                           "@behavior-decorated function"})
    ns: dict = {}
    try:
        exec(compile(source, "player_policy.py", "exec"), ns)
    except Exception as exc:
        return json.dumps({"ok": False, "error": f"{type(exc).__name__}: {exc}"})
    fns = [v for v in ns.values() if callable(v) and hasattr(v, "_behavior")]
    if not fns:
        return json.dumps({"ok": False, "error": "no @behavior-decorated function found"})
    _policy = fns[0]
    _robot = Robot(fns[0]._behavior["name"])
    _policy_error = None
    bus.emit("behavior", "wasm", f"policy loaded · {fns[0]._behavior['name']}", {})
    return json.dumps({"ok": True, "name": fns[0]._behavior["name"]})


def stop_policy() -> None:
    global _policy
    _policy = None
    get_arena().set_cmd(0.0, 0.0, 0.0)


def tick(state_json: str) -> str:
    """One 10 Hz step: feed arena state, run the policy, drain the event
    bus into the recorder. Returns what GET /api/arena/cmd would have."""
    global _policy, _policy_error, _last_level
    payload = json.loads(state_json)
    arena = get_arena()
    arena.update(payload)

    level = (payload.get("level") or {}).get("name", "")
    if level and level != _last_level:
        _last_level = level
        sightings.reset()
    sightings.observe(payload.get("detections") or [],
                      pose=payload.get("pose"), source="arena")

    rec = rec_mod.active_recorder()
    if rec is not None:
        # mirror routes/arena.py — Foxglove-native so the downloaded mcap
        # REPLAYS: oriented pose trail, lidar sweep, labeled detection
        # markers. Handle frame -> z-up world: fx=x, fy=-z, yaw=heading.
        pose = payload.get("pose") or {}
        fx, fy = pose.get("x", 0.0), -pose.get("z", 0.0)
        h = pose.get("heading", 0.0)
        alt = (pose.get("y", 0.0)
               if (payload.get("level") or {}).get("robot") == "drone" else 0.0)
        rec.write_pose(fx, fy, alt, heading=h)
        dets = payload.get("detections") or []
        if dets:
            rec.write_detections(dets, name="arena")
            rec.write_detection_scene(dets, fx, fy, h)
        lidar = payload.get("lidar") or []
        if lidar:
            rec.write_scan(lidar, fx, fy, h)
        q = rec._bus_queue
        while q is not None:
            try:
                rec.write_event(q.get_nowait())
            except queue.Empty:
                break

    if _policy is not None:
        try:
            _policy(_robot)
        except Exception:
            _policy_error = traceback.format_exc(limit=3)
            _policy = None
            arena.set_cmd(0.0, 0.0, 0.0)

    err, _policy_error = _policy_error, None
    return json.dumps({"cmd": arena.cmd(), "answer": arena.answer(),
                       "intent": arena.intent(),
                       "running": _policy is not None, "error": err})


def emit_event(type_: str, title: str, detail_json: str = "{}") -> None:
    bus.emit(type_ or "arena", "arena", title, json.loads(detail_json or "{}"))


def sightings_summary() -> str:
    return json.dumps(sightings.summary())


def record_start(robot_id: str = "arena") -> str:
    rec = rec_mod.start_recording(robot_id=robot_id)
    bus.emit("system", "recorder", f"RECORDING · {rec.run_id}", {"run": rec.run_id})
    return json.dumps(rec.status())


def record_stop() -> str:
    seal = rec_mod.stop_recording(do_anchor=False)
    if seal is None:
        return json.dumps({"ok": False, "error": "nothing is recording"})
    base = rec_mod.runs_root() / seal["robot_id"] / seal["run"]
    return json.dumps({"ok": True, "seal": seal,
                       "mcap_path": f"{base}.mcap",
                       "chain_path": f"{base}.chain.jsonl",
                       "seal_path": f"{base}.seal"})


def verify(mcap_path: str) -> str:
    return json.dumps(rec_mod.verify_mcap_run(mcap_path), default=str)
