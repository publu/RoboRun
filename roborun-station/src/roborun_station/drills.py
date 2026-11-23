"""The Stop drill matrix states: put a fake owner in a state, issue a hold, judge it from motion.

States (YAM station): idle, motion_streaming, jog, home (station Home move), thermal_blend, reset_home (the Home move
after a Reset's reopen), reset_reopen (arms let go while the drivers reopen). Metal owner: idle, motion_streaming, home.
`judge_hold` passes when, within the budget after the hold, every powered arm has stopped moving and its command
equals its measured pose (a hold from measured state, not an assumption). Everything runs on a SimClock.
"""
from __future__ import annotations

import math

from .core import Faults, SimClock
from .metal_owner import FakeMetalOwner
from .yam_station import FakeYamStation

YAM_STATES = ("idle", "motion_streaming", "jog", "home", "thermal_blend", "reset_home", "reset_reopen")
METAL_STATES = ("idle", "motion_streaming", "home")


def sweeping_predict(clock, arm_speed_rad_s=0.4, period=1/15):
    """A predict callable that keeps joint 1 moving (an motion lease mid-stream), within the station's bounds."""
    def predict(body):
        chunk = body["commanded"].get("q_l_chunk") or body["commanded"].get("q_r_chunk")
        target = list(chunk)
        target[0] += min(0.035, arm_speed_rad_s*period)
        return {"target": target, "acquired_at": clock(), "steps": 1, "inference_ms": 1.0, "frame_id": None}
    return predict


def yam_in_state(state: str, *, lease_free_hold=True, seed=0):
    clock = SimClock()
    station = FakeYamStation(clock=clock, faults=Faults(), lease_free_hold=lease_free_hold,
                             predict=sweeping_predict(clock), seed=seed)
    station.advance(0.5)
    lease_id = None
    if state == "motion_streaming":
        session = station.register_session(seconds=8)
        oid = station.observe()["observation_id"]
        lease_id = station.action({"observation_id": oid, "seconds": 8, "instruction": "pick up the cup",
                                   "reference": {"kind": "video_motion", "session_id": session}})["lease"]["id"]
        station.advance(1.0)
        station.heartbeat({"lease_id": lease_id})
    elif state == "jog":
        oid = station.observe()["observation_id"]
        target = list(station.left.q); target[0] += 0.08
        lease_id = station.action({"observation_id": oid, "seconds": 3, "instruction": "jog",
                                   "reference": {"kind": "mixed_jog", "target_rad": target}})["lease"]["id"]
        station.advance(0.03)
    elif state == "home":
        station.left.command(q=[v + 0.4 for v in station.left.spec.home[:1]] + list(station.left.spec.home[1:]))
        station.advance(2.0)
        station.home()
        station.advance(0.2)
    elif state == "thermal_blend":
        station.left.command(q=[v + 0.5 for v in station.left.spec.home[:1]] + list(station.left.spec.home[1:]))
        station.advance(2.0)
        station.thermal_retreat("left")
        station.advance(0.2)
    elif state in ("reset_home", "reset_reopen"):
        station.left.command(q=[v + 0.4 for v in station.left.spec.home[:1]] + list(station.left.spec.home[1:]))
        station.advance(2.0)
        station.reset()
        station.advance(0.5 if state == "reset_reopen" else 2.2)
    elif state != "idle":
        raise ValueError(f"unknown state {state}")
    return clock, station, lease_id


def metal_in_state(state: str, *, seed=7):
    clock = SimClock()
    owner = FakeMetalOwner(clock=clock, faults=Faults(), predict=None, seed=seed)
    owner.advance(0.5)
    lease_id = None
    if state == "motion_streaming":
        owner.predict = _metal_predict(clock)
        session = owner.register_session()
        oid = owner.motion_observe()["observation_id"]
        lease_id = owner.motion_action({"observation_id": oid, "seconds": 8,
                                    "reference": {"kind": "video_motion", "session_id": session}})["lease"]["id"]
        owner.advance(0.5)
        owner.motion_heartbeat({"lease_id": lease_id})
    elif state == "home":
        owner.arm.command(q=[owner.arm.spec.home[0] - 0.3] + list(owner.arm.spec.home[1:]))
        owner.advance(4.0)
        owner.home()
        owner.advance(0.3)
    elif state != "idle":
        raise ValueError(f"unknown state {state}")
    return clock, owner, lease_id


def _metal_predict(clock):
    def predict(body):
        chunk = list(body["commanded"]["q_r_chunk"])
        chunk[0] -= math.radians(0.15)
        return {"target": chunk, "acquired_at": clock(), "steps": 1, "inference_ms": 1.0, "frame_id": None}
    return predict


def judge_hold(owner, arms, *, budget_s=0.3, settle_tol_rad=1e-3):
    """Advance `budget_s` of sim time; then each powered arm must be still and commanded at its measured pose."""
    owner.advance(budget_s)
    result = {}
    for arm in arms:
        if not arm.powered:
            result[arm.spec.name] = {"holding": False, "why": "let go (reset reopen)"}
            continue
        before = list(arm.q)
        owner.advance(0.1)
        moved = max(abs(a-b) for a, b in zip(before, arm.q))
        lag = max(abs(a-b) for a, b in zip(arm.q_cmd, arm.q))
        result[arm.spec.name] = {"holding": moved <= settle_tol_rad and lag <= settle_tol_rad,
                                 "moved_rad": moved, "command_lag_rad": lag}
    return result
