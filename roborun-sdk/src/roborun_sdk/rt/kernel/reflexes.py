"""K5 contact reflexes and segment triggers.

Signals computed locally every tick from fresh samples:
- ``jaw``: measured jaw (segment units)
- ``bias_j1_4`` / ``bias_j5_6``: largest |commanded - measured| on joints 1-4 / 5-6 (the tracking pause signal)
- ``track_err``: largest |last written target - measured| over the joints
- ``effort_max``: largest |effort| when the owner reports fresh effort, else unavailable
- ``jaw_floor_gap``: jaw minus the segment's ``limits.jaw_floor`` when declared
Stats ``range`` / ``delta`` over a window ``w`` use the sample history. A condition on an unavailable signal is
false. Triggers are evaluated in order; the first match wins. Built-ins that no segment can switch off: the force
cap and the pack's tracking pause.
Never: open a jaw, lower a held object in mid-air, or unload (no ``unload`` primitive in this SDK version).
"""
from __future__ import annotations

import math

import numpy as np

from ..datatypes import ArmSample
from ..rest import History

OPS = {"<": lambda a, b: a < b, "<=": lambda a, b: a <= b, ">": lambda a, b: a > b, ">=": lambda a, b: a >= b,
       "==": lambda a, b: a == b, "!=": lambda a, b: a != b}


def _tool_points(arm_pack, q):
    geom = getattr(arm_pack, "geometry", None)
    if geom is None:
        return None
    last = max(c.link for c in geom.capsules)
    P0, P1, _ = geom.world(q)
    mask = np.array([c.link == last for c in geom.capsules])
    return np.vstack([P0[mask], P1[mask]])


def signals(sample: ArmSample, last_target, *, jaw_floor: float | None = None, arm_pack=None,
            min_gap_m: float | None = None) -> dict:
    q = np.asarray(sample.q, float)
    out = {"jaw": sample.jaw, "bias_j1_4": None, "bias_j5_6": None, "track_err": None, "effort_max": None,
           "effort": None, "jaw_effort": None, "jaw_floor_gap": None, "tool_z": None, "tool_lag": None,
           "min_gap_mm": None if min_gap_m is None else min_gap_m * 1000.0}
    tool = _tool_points(arm_pack, q)
    if tool is not None:
        out["tool_z"] = float(tool[:, 2].min())               # the lowest point of the last link (lift checks)
        cmd = sample.q_cmd if sample.q_cmd is not None else (None if last_target is None else list(last_target)[:len(q)])
        if cmd is not None:
            out["tool_lag"] = float(np.max(np.linalg.norm(_tool_points(arm_pack, np.asarray(cmd, float)) - tool, axis=1)))
    if sample.q_cmd is not None:
        bias = np.abs(np.asarray(sample.q_cmd, float) - q)
        out["bias_j1_4"] = float(bias[:4].max())
        out["bias_j5_6"] = float(bias[4:6].max()) if len(bias) > 4 else 0.0
    if last_target is not None:
        out["track_err"] = float(np.max(np.abs(np.asarray(last_target, float)[:len(q)] - q)))
    if sample.effort_fresh and sample.effort:
        out["effort_max"] = float(max(abs(v) for v in sample.effort))
        out["effort"] = float(max(abs(v) for v in sample.effort[:len(q)])) if len(sample.effort) >= len(q) else out["effort_max"]
        if len(sample.effort) > len(q):
            out["jaw_effort"] = float(abs(sample.effort[len(q)]))
    if jaw_floor is not None and sample.jaw is not None:
        out["jaw_floor_gap"] = float(sample.jaw - jaw_floor)
    return out


_COLUMN = {"jaw": None}


def _value(cond: dict, sigs: dict, history: History):
    sig = cond["sig"]
    stat = cond.get("stat", "value")
    if stat == "value":
        return sigs.get(sig)
    if sig not in _COLUMN:
        return None             # windowed stats are kept for the jaw only in this version
    w = float(cond.get("w", 0.4))
    return history.range_over(w, None) if stat == "range" else history.delta_over(w, None)


def holds(cond: dict, sigs: dict, history: History, fired: dict) -> bool:
    if "any" in cond:
        return any(holds(c, sigs, history, fired) for c in cond["any"])
    if "all" in cond:
        return all(holds(c, sigs, history, fired) for c in cond["all"])
    if "ref" in cond:
        return bool(fired.get(cond["ref"]))
    value = _value(cond, sigs, history)
    if value is None or not math.isfinite(value):
        return False
    return OPS[cond["op"]](value, float(cond["v"]))


def first_match(triggers, sigs: dict, history: History):
    fired: dict = {}
    match = None
    for trig in triggers:
        ok = holds(trig.when, sigs, history, fired)
        fired[trig.id] = ok
        if ok and match is None:
            match = trig
    return match


def builtin(arm_pack, sample: ArmSample) -> tuple[str | None, dict]:
    """Checks no segment can switch off: jaw force cap (when fresh effort exists) and the tracking pause."""
    if arm_pack.jaw.force_cap is not None and sample.effort_fresh and sample.effort:
        jaw_effort = abs(sample.effort[-1])
        if jaw_effort > arm_pack.jaw.force_cap:
            return "force_cap", {"effort": round(jaw_effort, 4), "cap": arm_pack.jaw.force_cap}
    if arm_pack.tracking_pause_rad is not None and sample.q_cmd is not None:
        bias = np.abs(np.asarray(sample.q_cmd, float) - np.asarray(sample.q, float))
        over = bias > arm_pack.tracking_pause_rad
        if over.any():
            j = int(np.argmax(bias - arm_pack.tracking_pause_rad))
            return "tracking_pause", {"joint": j + 1, "bias_rad": round(float(bias[j]), 4)}
    return None, {}
