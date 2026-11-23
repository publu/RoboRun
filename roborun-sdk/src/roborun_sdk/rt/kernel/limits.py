"""K3 hard limits on every write.

- Joint limits -0.01 rad (pack margin). A joint already past its limit may only move inward (as in the robot runtime's own check).
  "Past" is anchored on the *measured* pose and on the pack's Home only (the YAM rests at Home with joint 2 at
  -0.0013 rad, below its 0.01 rad soft limit; the robot runtime anchors the Metal's captured Home the same way). Never on a
  segment's planned rows: an anchor that moves with each plan lets chained segments ratchet a joint outward.
- Per-tick joint step (YAM 0.04 rad per 1/15 s) and the joint velocity cap.
- Per-point clamp: 20 mm per 1/15 s on every capsule endpoint (exact FK when the pack carries a chain; otherwise the
  conservative bound sum_j reach_j * |dq_j|, which over-approximates every point's travel). EE speed rides on the same
  per-point measure: min(pack, segment, grant, 0.10 m/s on WSS); 0.04 base, free_max with a segment, and
  loaded_max while K8 ``held`` is set.
- Jaw: range, step 0.04 per 1/15 s, a request never leads the measured jaw by more than 0.03 (except to keep a squeeze
  the owner already holds), and never opens past the segment's jaw reference (only a release opens the jaw; no
  release is played in this version). A write with no jaw reading is refused.
Clamps scale the whole step (direction preserved). Any clamp in normal operation is reported as a planner bug.
``WriteCheck.scale`` is the fraction of the proposed joint step the speed clamps (joint step, per-point / EE speed)
let through: the player advances executed time by that fraction only, so a slowed segment still follows its rows to
the last one. Limit and jaw clamps do not scale time (the target simply cannot go there).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from ..datatypes import ArmSample
from ..pack import ArmPack

TICK_15HZ = 1 / 15


@dataclass
class WriteCheck:
    target: np.ndarray
    clamps: list = field(default_factory=list)
    refused: str | None = None
    detail: dict = field(default_factory=dict)
    scale: float = 1.0                  # fraction of the proposed joint step the speed clamps let through


def joint_bounds(arm: ArmPack, q_measured) -> tuple[np.ndarray, np.ndarray]:
    """Soft limits widened only to where the arm *is* (measured) and to the pack's Home: a joint past its limit may
    hold or move inward, never further out."""
    q = np.asarray(q_measured, float)
    lo = np.minimum(arm.soft_lower(), q)
    hi = np.maximum(arm.soft_upper(), q)
    if arm.home_q is not None:
        lo = np.minimum(lo, arm.home_q)
        hi = np.maximum(hi, arm.home_q)
    return lo, hi


def ee_cap(arm: ArmPack, seg_limits: dict, grant_cap: float | None, transport: str, loaded: bool = False) -> float:
    """EE / per-point speed cap. ``arm.tcp_m_s`` holds effective values (pack.py writes the defaults in)."""
    tcp = arm.tcp_m_s
    if "ee_speed_m_s" in seg_limits:
        caps = [seg_limits["ee_speed_m_s"], tcp["loaded_max" if loaded else "free_max"]]
    else:
        caps = [tcp["base"]]
    if loaded:
        caps.append(tcp["loaded_max"])
    if grant_cap is not None:
        caps.append(grant_cap)
    if transport == "wss":
        caps.append(0.10)
    return float(min(caps))


def point_travel(arm: ArmPack, qa, qb) -> float:
    if arm.geometry is not None:
        return arm.geometry.max_point_displacement(qa, qb)
    if arm.point_reach_m is not None:
        return float(np.sum(arm.point_reach_m * np.abs(np.asarray(qb) - np.asarray(qa))))
    return math.inf


def jaw_bounds(arm: ArmPack, *, measured_jaw: float, jaw_ref: float | None, jaw_allow: float | None) -> tuple[float, float]:
    """(most-closed, most-open) jaw a write may request, in the arm's jaw units. Most-open: the segment's jaw reference
    plus its noise allowance (``jaw_ref`` None: the measured jaw, no allowance). Most-closed: 0.03 (lead_max) past the
    measured jaw, or the reference when the owner already squeezes deeper than that."""
    d = arm.jaw.opening_direction()
    o_lo, o_hi = sorted((arm.jaw.lo * d, arm.jaw.hi * d))
    ref = measured_jaw if jaw_ref is None else jaw_ref
    allow = 0.0 if jaw_allow is None else max(0.0, float(jaw_allow))
    open_max = min(o_hi, ref * d + allow)
    close_min = max(o_lo, min(measured_jaw * d - arm.jaw.lead_max, ref * d))
    return close_min * d, open_max * d


def check_target(arm: ArmPack, target7, *, last7, measured: ArmSample, dt: float, seg_limits: dict | None = None,
                 grant_cap: float | None = None, transport: str = "local", loaded: bool = False,
                 jaw_ref: float | None = None, jaw_allow: float | None = None) -> WriteCheck:
    seg_limits = seg_limits or {}
    n = arm.n
    target = np.asarray(target7, float).copy()
    if target.shape != (n + 1,) or not np.isfinite(target).all():
        return WriteCheck(target, refused="nonfinite")
    if measured.jaw is None or not math.isfinite(measured.jaw):
        return WriteCheck(target, refused="jaw_unknown", detail={"why": "no jaw reading: the jaw bounds are unknown"})
    q = np.asarray(measured.q, float)
    base = np.asarray(last7, float)[:n] if last7 is not None else q
    out = WriteCheck(target)
    # joint limits, inward-only beyond them (anchored on the measured pose and the pack's Home only)
    lo, hi = joint_bounds(arm, q)
    clipped = np.clip(target[:n], lo, hi)
    if np.any(np.abs(clipped - target[:n]) > 1e-12):
        joints = [int(i) + 1 for i in np.nonzero(np.abs(clipped - target[:n]) > 1e-12)[0]]
        out.clamps.append("joint_limit")
        out.detail["joint_limit"] = joints
        target[:n] = clipped
    # per-tick step and velocity
    step = target[:n] - base
    worst = float(np.max(np.abs(step))) if n else 0.0
    step_cap = min(arm.step_rad_per_15hz * dt / TICK_15HZ, arm.vel_rad_s * dt,
                   seg_limits.get("joint_speed_rad_s", math.inf) * dt)
    if worst > step_cap + 1e-12:
        step *= step_cap / worst
        out.scale *= step_cap / worst
        out.clamps.append("joint_step")
        out.detail["joint_step_rad"] = round(worst, 5)
    # per-point clamp and EE speed on capsule endpoints
    point_cap = min(arm.point_step_m_per_15hz * dt / TICK_15HZ,
                    seg_limits.get("point_step_m", math.inf) * dt / TICK_15HZ,
                    ee_cap(arm, seg_limits, grant_cap, transport, loaded) * dt)
    travel = point_travel(arm, base, base + step)
    if travel > point_cap + 1e-12:
        if not math.isfinite(travel):
            return WriteCheck(target, refused="no_point_bound", detail={"why": "pack has neither a chain nor point_reach_m"})
        scale = point_cap / travel
        for _ in range(8):                      # FK travel is near-linear for small steps; confirm by re-evaluation
            if point_travel(arm, base, base + step * scale) <= point_cap + 1e-12:
                break
            scale *= 0.9
        step *= scale
        out.scale *= scale
        out.clamps.append("point_step")
        out.detail["point_travel_mm"] = round(travel * 1000, 2)
        out.detail["point_cap_mm"] = round(point_cap * 1000, 2)
    target[:n] = base + step
    # jaw: step from the last command, then the bounds (the never-open bound always wins)
    jaw = target[n]
    d = arm.jaw.opening_direction()
    close_lim, open_lim = jaw_bounds(arm, measured_jaw=float(measured.jaw), jaw_ref=jaw_ref, jaw_allow=jaw_allow)
    last_jaw = float(np.asarray(last7, float)[n]) if last7 is not None else math.nan
    if not math.isfinite(last_jaw):
        last_jaw = float(measured.jaw if jaw_ref is None else jaw_ref)
    jaw_step = arm.jaw.step_per_15hz * dt / TICK_15HZ
    t = min(max(jaw * d, last_jaw * d - jaw_step), last_jaw * d + jaw_step)
    t = min(max(t, close_lim * d), open_lim * d)
    j = t * d
    if abs(j - jaw) > 1e-9:
        out.clamps.append("jaw")
        out.detail["jaw_requested"] = round(float(jaw), 5)
    target[n] = j
    out.target = target
    return out
