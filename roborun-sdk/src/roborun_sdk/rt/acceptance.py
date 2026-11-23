"""Segment acceptance. Every rule must pass; otherwise a typed refusal and the arm keeps holding.

1 protocol (wire, signature, seq, generation; duplicates are re-acked by the runtime, never re-run)
2 context (pack, calibration, plan basis newer than the last scene-change mark)
3 start (start_q within tol, at rest by the 0.3 s / 1.5 mrad window, jaw known and within tol, samples <= 50 ms; or
  a continuation from the named segment's end)
4 other arms in their stated state with fresh samples
5 interlocks and health (armed, no latched Stop/fault/link loss, motors below the refuse-start threshold, the jaw opens
  only in a release, nothing while pending_release)
6 trajectory (limits -0.01 rad inward-only from the *measured* pose and the pack's Home, per-tick steps,
  per-point/EE speed (loaded_max while held), jaw range, never more open than the jaw reference, swept proximity
  from the measured pose through every row)
7 tightening only (segment limits, rest test and jaw_tol never looser than the pack's)
8 expiry (expires_after_s from receipt)
H one moving arm per rig.

The jaw reference is the most-closed of the measured jaw, the owner's commanded jaw (holding an object, the command
squeezes past the measured width; opening up to the measured width would release the squeeze) and, while K8 ``held``
is set, the jaw recorded at grasp. A ``move`` may close the jaw but never open it past the reference by more than
min(jaw_tol, the pack's rest jaw_range) of noise; while ``held`` that is refused as ``held_interlock`` whatever the
segment's label. A segment never starts with the jaw reading missing.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .datatypes import ArmSample, Refused
from .kernel.limits import ee_cap, joint_bounds, point_travel
from .pack import ArmPack, Pack
from .segment import Segment

START_SAMPLE_MAX_AGE_S = 0.050
TICK_15HZ = 1 / 15


@dataclass
class AcceptanceContext:
    now: float
    received_t: float
    pack: Pack
    samples: dict                       # arm -> ArmSample | None (newest)
    histories: dict                     # arm -> History
    descriptors: dict                   # arm -> ArmDescriptor
    proximity: object                   # CapsuleProximity | MixedSafetyProximity
    interlocks: object                  # Interlocks
    health: object                      # HealthMonitor
    stopped: bool
    link_accepting: bool
    link_latched_lost: bool
    armed: bool
    last_seq: int
    generations: dict                   # task -> highest generation seen
    calibration: dict                   # {"id": str|None, "suspect": bool}
    scene_mark_t: float
    observations: dict                  # obs id -> sdk_t of the capture
    trust_unknown_plan_basis: bool
    grants: dict                        # arm -> Grant
    finished: dict                      # segment id -> {"outcome", "end_q", "end_jaw", "t"}
    signature_ok: bool
    transport: str = "local"
    extra: dict = field(default_factory=dict)


def _sample_age(s: ArmSample, now: float) -> float:
    return now - s.t + s.clock_uncertainty_s


def jaw_reference(arm: ArmPack, s: ArmSample, held: dict | None) -> float:
    """The most-closed of the measured jaw, the owner's commanded jaw and (while ``held``) the jaw at grasp."""
    vals = [float(s.jaw)]
    if s.jaw_cmd is not None and math.isfinite(s.jaw_cmd):
        vals.append(float(s.jaw_cmd))
    if held and isinstance(held.get("jaw"), (int, float)) and not isinstance(held.get("jaw"), bool) \
            and math.isfinite(held["jaw"]):
        vals.append(float(held["jaw"]))
    return min(vals) if arm.jaw.opening_direction() > 0 else max(vals)


def rest_spec(seg: Segment, pack: Pack) -> dict:
    """The rest test a segment may only tighten: a shorter window or a wider range than
    the pack's is a loosening and is refused."""
    pr = pack.rest
    sr = seg.valid_if.get("rest") or {}
    window = float(sr["window_s"]) if sr.get("window_s") is not None else float(pr["window_s"])
    rng = float(sr["range_rad"]) if sr.get("range_rad") is not None else float(pr["range_rad"])
    jaw = float(sr["jaw_range"]) if sr.get("jaw_range") is not None else float(pr["jaw_range"])
    if window < float(pr["window_s"]) - 1e-12:
        raise Refused("loosening_limits", f"valid_if.rest.window_s {window} is shorter than the pack's {pr['window_s']}",
                      key="rest.window_s")
    if rng > float(pr["range_rad"]) + 1e-12:
        raise Refused("loosening_limits", f"valid_if.rest.range_rad {rng} is wider than the pack's {pr['range_rad']}",
                      key="rest.range_rad")
    if jaw > float(pr["jaw_range"]) + 1e-12:
        raise Refused("loosening_limits", f"valid_if.rest.jaw_range {jaw} is wider than the pack's {pr['jaw_range']}",
                      key="rest.jaw_range")
    return {"window_s": max(window, float(pr["window_s"])), "range_rad": min(rng, float(pr["range_rad"])),
            "jaw_range": min(jaw, float(pr["jaw_range"]))}


def _limits_tighter(seg: Segment, arm: ArmPack, loaded: bool):
    lim = seg.limits
    caps = {"ee_speed_m_s": arm.tcp_m_s["loaded_max" if loaded else "free_max"],
            "point_step_m": arm.point_step_m_per_15hz, "joint_speed_rad_s": arm.vel_rad_s,
            "jaw_step": arm.jaw.step_per_15hz}
    if arm.jaw.force_cap is not None:
        caps["force_cap"] = arm.jaw.force_cap
    for key, value in lim.items():
        if key in caps and value > caps[key] + 1e-12:
            why = " while an object is held (loaded_max)" if key == "ee_speed_m_s" and loaded else ""
            raise Refused("loosening_limits", f"segment {key} {value} exceeds the pack's {caps[key]}{why}", key=key)
    jaw_tol = float(seg.valid_if.get("jaw_tol", 0.005))
    if jaw_tol > arm.jaw.lead_max + 1e-12:
        raise Refused("loosening_limits", f"valid_if.jaw_tol {jaw_tol} exceeds the pack's jaw lead_max {arm.jaw.lead_max}",
                      key="valid_if.jaw_tol")


def _trajectory(seg: Segment, arm: ArmPack, start_q: np.ndarray, jaw_ref: float, jaw_allow: float, held,
                loaded: bool, ctx: AcceptanceContext):
    rows = seg.rows
    n = arm.n
    if rows.shape[1] != n + 1:
        raise Refused("semantics_mismatch", f"rows have {rows.shape[1]} columns; {arm.semantics} needs {n + 1}")
    q = rows[:, :n]
    # joint limits -margin; a joint already past its limit at the measured start (or resting at the pack's Home past
    # it) may only hold or move inward. Never anchored on the planned rows: that anchor would move with each plan.
    lo, hi = joint_bounds(arm, start_q)
    bad = np.nonzero((q < lo - 1e-9) | (q > hi + 1e-9))
    if len(bad[0]):
        r, j = int(bad[0][0]), int(bad[1][0])
        raise Refused("joint_limit", f"row {r} joint {j + 1} at {q[r, j]:.4f} rad is outside the pack limits "
                      f"(inward-only from the measured {start_q[j]:.4f})", row=r, joint=j + 1)
    jaw = rows[:, n]
    if np.any(jaw < arm.jaw.lo - 1e-9) or np.any(jaw > arm.jaw.hi + 1e-9):
        raise Refused("jaw_range", "jaw column outside the pack's jaw range")
    opening = (jaw - jaw_ref) * arm.jaw.opening_direction()
    if np.any(opening > jaw_allow + 1e-9):
        r = int(np.argmax(opening))
        detail = {"row": r, "jaw": round(float(jaw[r]), 5), "reference": round(float(jaw_ref), 5),
                  "allowance": round(float(jaw_allow), 5)}
        if held is not None:
            raise Refused("held_interlock", "the jaw would open past its present command while an object is held "
                          "(only an accepted release opens it, whatever the segment's label)", **detail)
        raise Refused("jaw_open_not_release", "only a release may open the jaw", **detail)
    if seg.format == "rows30.v1":
        steps = np.abs(np.diff(q, axis=0))
        step_cap = min(arm.step_rad_per_15hz * seg.dt / TICK_15HZ, arm.vel_rad_s * seg.dt,
                       seg.limits.get("joint_speed_rad_s", math.inf) * seg.dt)
        if steps.size and float(steps.max()) > step_cap + 1e-9:
            r = int(np.unravel_index(np.argmax(steps), steps.shape)[0])
            raise Refused("step_limit", f"row {r}->{r + 1} steps {float(steps.max()):.4f} rad > {step_cap:.4f} per tick",
                          row=r)
        cap = min(arm.point_step_m_per_15hz * seg.dt / TICK_15HZ,
                  seg.limits.get("point_step_m", math.inf) * seg.dt / TICK_15HZ,
                  ee_cap(arm, seg.limits, None, ctx.transport, loaded) * seg.dt)
        for r in range(len(q) - 1):
            travel = point_travel(arm, q[r], q[r + 1])
            if travel > cap + 1e-9:
                reason = "no_point_bound" if not math.isfinite(travel) else "ee_speed"
                raise Refused(reason, f"row {r}->{r + 1} moves a capsule point {travel * 1000:.1f} mm in one tick "
                              f"(cap {cap * 1000:.2f} mm)", row=r)
    jaw_d = np.abs(np.diff(jaw))
    jaw_cap = arm.jaw.step_per_15hz * seg.dt / TICK_15HZ if seg.format == "rows30.v1" else math.inf
    if jaw_d.size and float(jaw_d.max()) > jaw_cap + 1e-9:
        raise Refused("jaw_step", "jaw rows step faster than the pack allows")


def check(seg: Segment, ctx: AcceptanceContext) -> dict:
    """Raise Refused, or return the start facts for the ack (``start_measured``) and the playback (``jaw``)."""
    raw = seg.raw
    # 1. protocol
    if raw.get("wire") != 1:
        raise Refused("wire_mismatch", "wire version must be exactly 1")
    if not ctx.signature_ok:
        raise Refused("bad_signature", "segment signature did not verify against the pack's trusted keys")
    if seg.seq <= ctx.last_seq:
        raise Refused("stale_seq", f"seq {seg.seq} is not above {ctx.last_seq}")
    if seg.generation < ctx.generations.get(seg.task, -1):
        raise Refused("old_generation", f"generation {seg.generation} is below the task's current one")
    # 8. expiry (cheap; before any geometry)
    if ctx.now - ctx.received_t > float(seg.valid_if.get("expires_after_s", 10)):
        raise Refused("expired", "segment expired before it could start")
    # arm, semantics
    if seg.arm not in ctx.pack.arms or seg.arm not in ctx.descriptors:
        raise Refused("unknown_arm", f"arm {seg.arm!r} is not connected or not in the pack")
    arm = ctx.pack.arm(seg.arm)
    desc = ctx.descriptors[seg.arm]
    if seg.semantics != arm.semantics or seg.semantics != desc.semantics:
        raise Refused("semantics_mismatch", f"segment {seg.semantics} vs pack {arm.semantics} vs connector {desc.semantics}")
    if seg.format not in desc.formats:
        raise Refused("unsupported_format", f"connector {seg.arm} does not take {seg.format}")
    # 5. interlocks and health (state that holds everything)
    if ctx.stopped:
        raise Refused("stopped", "Stop is latched; clearing it takes an operator action")
    faults = ctx.health.faults()
    if faults:
        raise Refused("fault_latched", "a latched arm fault holds the rig", faults=faults)
    if ctx.link_latched_lost:
        raise Refused("link_lost", "link loss is latched until a new session")
    if not ctx.link_accepting:
        raise Refused("link_late", "the link is late; nothing new is accepted")
    if not ctx.armed:
        raise Refused("not_armed", "both arming halves are required")
    # H. one moving arm per rig
    for other, grant in ctx.grants.items():
        if grant is not None:
            raise Refused("another_arm_moving", f"{other} holds a grant (one moving arm per rig in v1)")
    held = ctx.interlocks.held(seg.arm)
    loaded = held is not None
    # 7. tightening only
    _limits_tighter(seg, arm, loaded)
    rest = rest_spec(seg, ctx.pack)
    # 2. context
    context = seg.context
    if context.get("pack") is not None and context.get("pack") != ctx.pack.digest:
        raise Refused("pack_mismatch", "segment was planned against another pack",
                      segment_pack=context.get("pack"), active=ctx.pack.digest)
    if context.get("pack") is None and ctx.pack.signed:
        raise Refused("pack_mismatch", "segment names no pack")
    cal = context.get("calibration")
    if ctx.calibration.get("id") is not None and cal != ctx.calibration["id"]:
        raise Refused("calibration_mismatch", f"segment calibration {cal!r} is not the current {ctx.calibration['id']!r}")
    if ctx.calibration.get("suspect"):
        raise Refused("calibration_suspect", "the current calibration is suspect (camera identity changed)")
    basis = context.get("plan_basis_obs")
    if basis is not None:
        obs_t = ctx.observations.get(basis)
        if obs_t is None and not ctx.trust_unknown_plan_basis:
            raise Refused("plan_basis_unknown", f"observation {basis!r} was not captured by this SDK")
        if obs_t is not None and obs_t <= ctx.scene_mark_t:
            raise Refused("plan_basis_stale", "the plan predates the last scene change (Stop, link loss, operator action, "
                          "proximity hold, fault or reconnect)")
    elif not ctx.trust_unknown_plan_basis:
        raise Refused("plan_basis_unknown", "segment names no plan basis observation")
    # 3. start
    s = ctx.samples.get(seg.arm)
    if s is None:
        raise Refused("sample_missing", f"no sample for {seg.arm}")
    age = _sample_age(s, ctx.now)
    if age > START_SAMPLE_MAX_AGE_S:
        raise Refused("sample_stale", f"{seg.arm} sample is {age * 1000:.1f} ms old (<= 50 ms)", age_ms=round(age * 1000, 1))
    if s.jaw is None or not math.isfinite(s.jaw):
        raise Refused("jaw_unknown", f"{seg.arm} reports no jaw reading: a segment never starts with the jaw unknown")
    q = np.asarray(s.q, float)
    if ctx.interlocks.pending_release(seg.arm):
        raise Refused("pending_release", "only a release-escape segment is accepted (none in this SDK version)")
    tol = float(seg.valid_if.get("tol_rad", 0.01))
    if seg.after is not None:
        prev = ctx.finished.get(seg.after["segment"])
        if prev is None:
            raise Refused("continuation_unknown", f"{seg.after['segment']} has not finished here")
        want = seg.after.get("outcome")
        if want is not None and prev["outcome"] != want:
            raise Refused("continuation_outcome", f"{seg.after['segment']} ended {prev['outcome']}, not {want}")
        start_q = np.asarray(prev["end_q"], float)
    else:
        start_q = np.asarray(seg.valid_if["start_q"], float)
    err = np.abs(q - start_q)
    if float(err.max()) > tol + 1e-12:
        j = int(np.argmax(err))
        raise Refused("start_state_mismatch", f"joint j{j + 1} is {float(err[j]):.4f} rad from start_q",
                      joint=f"j{j + 1}", err_rad=round(float(err[j]), 4), tol_rad=tol)
    rows0 = seg.rows[0, :arm.n]
    if float(np.max(np.abs(rows0 - q))) > tol + 1e-12:
        raise Refused("start_state_mismatch", "the first row is not at the measured start")
    ok, rest_detail = ctx.histories[seg.arm].at_rest(ctx.now, window_s=rest["window_s"], range_rad=rest["range_rad"],
                                                     jaw_range=rest["jaw_range"], max_age_s=START_SAMPLE_MAX_AGE_S)
    if not ok:
        raise Refused("not_at_rest", "the arm is not at rest by the position window", **rest_detail)
    jaw_tol = float(seg.valid_if.get("jaw_tol", 0.005))
    jaw_ref = jaw_reference(arm, s, held)
    jaw_allow = min(jaw_tol, rest["jaw_range"])
    want_jaw = seg.valid_if.get("jaw")
    if want_jaw is not None and abs(s.jaw - float(want_jaw)) > jaw_tol + 1e-12:
        raise Refused("jaw_mismatch", f"jaw {s.jaw} is not within tol of {want_jaw}")
    if abs(seg.rows[0, arm.n] - jaw_ref) > jaw_tol + 1e-12:
        raise Refused("jaw_mismatch", "the first row's jaw is not the jaw's present command",
                      first_row=round(float(seg.rows[0, arm.n]), 5), reference=round(jaw_ref, 5))
    # 5. thermal
    th = ctx.health.arms[seg.arm]
    if th.thermal != "ok":
        raise Refused("too_hot", f"{seg.arm} motors at {th.temp_max_c} C refuse a start", temp_max_c=th.temp_max_c)
    # 4. other arms
    others = seg.valid_if.get("others") or {}
    for name, s_other in ctx.samples.items():
        if name == seg.arm:
            continue
        if s_other is None or not ctx.health.fresh(name):
            raise Refused("other_arm_unknown", f"{name} has no fresh sample: an offline arm is an obstacle of unknown pose")
        want = others.get(name)
        if want:
            state = want.get("state")
            if state is not None and s_other.status != state and not (state == "parked" and s_other.status in ("parked", "holding", "idle")):
                raise Refused("other_arm_state", f"{name} is {s_other.status}, segment needs {state}")
            if want.get("q") is not None:
                if float(np.max(np.abs(np.asarray(s_other.q) - np.asarray(want["q"], float)))) > float(want.get("tol_rad", 0.01)):
                    raise Refused("other_arm_state", f"{name} moved since planning")
    # 6. trajectory (the measured pose anchors the limits; the jaw reference anchors the jaw)
    _trajectory(seg, arm, q, jaw_ref, jaw_allow, held, loaded, ctx)
    prox = ctx.proximity.check_path(seg.arm, seg.rows, ctx.samples, ctx.now)
    if not prox.ok:
        detail = {("why" if k == "reason" else k): v for k, v in prox.public().items()
                  if k in ("reason", "min_gap_m", "required_m", "table_m", "obstacle_m", "checker")}
        raise Refused("proximity" if prox.reason != "stale" else "proximity_stale", prox.message or "swept proximity refused",
                      **detail)
    return {"start_measured": {seg.arm: {"q": [round(v, 4) for v in s.q], "jaw": s.jaw, "age_ms": round(age * 1000, 1)}},
            "rest": rest_detail, "proximity": prox.public(),
            "jaw": {"reference": jaw_ref, "allowance": jaw_allow, "held": loaded}}
