"""Per-run telemetry series for the Analyze view (Antioch parity).

Reads a sealed run's MCAP into the time series the run-detail panels draw:
trajectory (the actual path), velocity + commanded action, obstacle clearance,
and the latest lidar sweep. Pure read over the recorded channels — `/pose`,
`/cmd`, `/scan`, `/telemetry/*` — so every panel is backed by the sealed record.
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from roborun.events import runs_root


def _find_mcap(run_id: str, robot_id: str | None = None) -> Path | None:
    root = runs_root()
    if robot_id:
        p = root / robot_id / f"{run_id}.mcap"
        return p if p.exists() else None
    hits = list(root.glob(f"*/{run_id}.mcap"))
    return hits[0] if hits else None


def run_series(run_id: str, robot_id: str | None = None) -> dict[str, Any]:
    """Extract the panels' series from a run. {ok, trajectory, velocity, cmd,
    clearance, scan, t0, t1, counts}."""
    mcap = _find_mcap(run_id, robot_id)
    if mcap is None:
        return {"ok": False, "error": f"run {run_id} not found"}
    from mcap.reader import make_reader

    traj: list[list[float]] = []
    velocity: list[dict] = []
    cmd: list[dict] = []
    clearance: list[dict] = []
    scan_latest: list[float] = []
    t0 = t1 = None
    counts: dict[str, int] = {}

    with open(mcap, "rb") as fh:
        for _s, channel, message in make_reader(fh).iter_messages():
            if channel.message_encoding != "json":
                continue
            try:
                obj = json.loads(message.data)
            except Exception:
                continue
            ts = message.log_time / 1e9
            t0 = ts if t0 is None else min(t0, ts)
            t1 = ts if t1 is None else max(t1, ts)
            topic = channel.topic
            counts[topic] = counts.get(topic, 0) + 1
            if topic in ("/pose", "/odom"):
                p = (obj.get("pose") or {}).get("position") or {}
                if "x" in p:
                    traj.append([round(p.get("x", 0), 3), round(p.get("z", 0), 3)])
            elif topic == "/cmd":
                cmd.append({"t": round(ts, 3), "forward": obj.get("forward", 0),
                            "turn": obj.get("turn", 0)})
            elif topic.startswith("/telemetry/velocity"):
                d = obj.get("data") or {}
                velocity.append({"t": round(ts, 3), "linear": d.get("x", 0),
                                 "angular": d.get("angular_z", 0)})
            elif topic == "/scan":
                ranges = [r for r in (obj.get("ranges") or []) if isinstance(r, (int, float)) and r > 0]
                if ranges:
                    clearance.append({"t": round(ts, 3), "min": round(min(ranges), 3)})
                    scan_latest = [round(r, 3) for r in obj.get("ranges") or []]

    # velocity falls back to commanded action if no /telemetry/velocity channel
    if not velocity and cmd:
        velocity = [{"t": c["t"], "linear": c["forward"], "angular": c["turn"]} for c in cmd]

    return {"ok": True, "run": run_id, "robot_id": robot_id or mcap.parent.name,
            "trajectory": traj, "velocity": velocity, "cmd": cmd,
            "clearance": clearance, "scan": scan_latest,
            "t0": t0, "t1": t1, "duration_s": round((t1 - t0), 2) if t0 and t1 else 0,
            "counts": counts}
