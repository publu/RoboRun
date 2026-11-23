"""Uplink messages in manifest.wire.v1's typed vocabulary (the wire schema package).

roborun-rt keeps a richer local vocabulary (outcomes such as ``fresh_pose_budget_exhausted``, events such as
``scene_change_mark``). The link translates every uplink message to the wire's types before sending: refusal reasons
to ``RefusalReason``, events to ``EventKind``, gate outcomes to the six wire outcomes, Stop answers to ``StopArm``
rows, causes to ``Code`` strings. The local value always travels along in ``detail`` so nothing is lost.
When the wire schema package is importable every outgoing message is validated against its model; a message that would not
validate is replaced by a ``refusal`` event naming the problem (never silently dropped).
"""
from __future__ import annotations

import math
import re

from .. import SDK_NAME, WIRE_VERSION
from ..rt import wire

REASONS = {
    "bad_signature": "signature_invalid", "unsigned": "signature_invalid", "wire_mismatch": "wire_mismatch",
    "stale_seq": "seq_regression", "old_generation": "generation_regression", "pack_mismatch": "pack_mismatch",
    "calibration_mismatch": "calibration_mismatch", "calibration_suspect": "calibration_suspect",
    "plan_basis_stale": "plan_basis_stale", "plan_basis_unknown": "plan_basis_stale",
    "semantics_mismatch": "semantics_mismatch", "unknown_arm": "semantics_mismatch",
    "unsupported_format": "unsupported_format", "unsupported_kind": "unsupported_format",
    "unsupported_branches": "unsupported_format", "unsupported_player": "unsupported_format",
    "unsupported_player_feature": "unsupported_format", "start_state_mismatch": "start_state_mismatch",
    "not_at_rest": "not_at_rest", "jaw_mismatch": "jaw_mismatch", "jaw_unknown": "jaw_mismatch",
    "unsupported_gate": "unsupported_format", "sample_stale": "stale_sample",
    "sample_missing": "stale_sample", "continuation_unknown": "after_mismatch", "continuation_outcome": "after_mismatch",
    "other_arm_state": "other_arm_state", "other_arm_unknown": "arm_offline", "not_armed": "not_armed",
    "stopped": "stopped_latched", "fault_latched": "fault_latched", "link_lost": "link_lost_latched",
    "link_late": "link_late", "too_hot": "thermal", "held_interlock": "interlock_held",
    "pending_release": "interlock_pending_release", "nonfinite": "trajectory_invalid", "malformed": "trajectory_invalid",
    "jaw_step": "trajectory_invalid", "jaw_open_not_release": "trajectory_invalid", "joint_limit": "joint_limit",
    "step_limit": "joint_step", "ee_speed": "ee_speed", "no_point_bound": "point_step", "jaw_range": "jaw_range",
    "proximity": "proximity", "proximity_stale": "proximity", "keep_in": "keep_in", "loosening_limits": "limits_loosened",
    "expired": "expired", "unsupported_trigger_signal": "unknown_signal", "unsupported_trigger_stat": "unknown_signal",
    "unsupported_trigger_action": "trajectory_invalid", "unsupported_trigger_then": "trajectory_invalid",
    "another_arm_moving": "busy", "connector_not_wired": "connector_not_wired", "connector_refused": "busy",
}   # link_late and connector_not_wired have their own wire reasons
EVENTS = {"trigger": "trigger", "arm_offline": "arm_offline", "link_late": "link_late", "link_lost": "link_lost",
          "link_back": "link_back", "stop_cleared": "stop_cleared", "fresh_pose_hold": "fresh_pose_hold",
          "tick_overrun": "tick_overrun", "tick_late": "tick_overrun", "thermal": "thermal", "clamp": "clamp",
          "pack_activated": "pack_activated", "pack_staged": "pack_staged", "proximity_hold": "refusal",
          "limit_refused": "refusal", "reflex": "trigger", "connector_error": "fault", "control_refused": "refusal",
          "decide_refused": "refusal", "capture_refused": "refusal", "arm_stale": "fresh_pose_hold"}
LOCAL_ONLY = {"scene_change_mark", "segment_started", "control_done"}
HEALTH = {"ok": "ok", "degraded": "degraded", "fault": "fault", "offline": "offline", "stale": "degraded",
          "unknown": "degraded"}
CONNECTOR_KIND = {"yam-linear-4310": "yam-station", "makermods-metal": "metal-owner"}
_CODE = re.compile(r"[^a-z0-9_]+")


def code(text) -> str:
    """A wire ``Code``: lower snake case, at most 64 characters, starting with a letter."""
    out = _CODE.sub("_", str(text).lower()).strip("_")[:63] or "unknown"
    return out if out[0].isalpha() else "c_" + out[:61]


def _detail(obj, depth=0):
    """Bounded JSON (manifest_wire Detail: <= 64 keys, depth <= 8, finite numbers, arrays <= 256)."""
    if depth > 6:
        return str(obj)[:200]
    if isinstance(obj, dict):
        return {str(k)[:64]: _detail(v, depth + 1) for k, v in list(obj.items())[:60]}
    if isinstance(obj, (list, tuple)):
        return [_detail(v, depth + 1) for v in list(obj)[:256]]
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if obj is None or isinstance(obj, (bool, int, str)):
        return obj
    return str(obj)[:200]


def _valid(msg: dict) -> dict:
    if not wire.available():
        return msg
    try:
        wire.model(msg, msg.get("type", ""))
        return msg
    except ValueError as exc:
        return {"type": "event", "seq": msg.get("seq", 0), "sdk_t": float(msg.get("sdk_t") or 0.0), "kind": "refusal",
                "detail": {"why": "uplink_not_wire_valid", "type": msg.get("type"), "error": str(exc)[:200]}}


def hello(info: dict, *, rig: str, device: str, seq: int, cameras: dict | None = None,
          stop_sources=("lan", "cli", "api")) -> dict:
    arms = {}
    for name, a in info["arms"].items():
        conn = a["connector"]
        arms[name] = {"semantics": a["semantics"], "joints": list(a["joints"]),
                      "connector": {"kind": CONNECTOR_KIND.get(a.get("embodiment"), "sim"), "version": SDK_NAME.split("/")[1],
                                    "control_hz": conn["control_hz"], "command_hz": conn["command_hz"],
                                    "formats": list(conn["formats"]), "hold": "position",
                                    # never claim a hold nobody measured: unknown is reported as the worst case
                                    "on_command_gap": conn["on_command_gap"] if conn["on_command_gap"] != "unknown" else "disable"},
                      "adapter": {k: v for k, v in a["adapter"].items() if k in ("kind", "serial", "channel", "mode")},
                      "health": HEALTH.get(a.get("health") or "unknown", "degraded"),
                      "interlocks": {"held": bool((a.get("interlocks") or {}).get("held")),
                                     "pending_release": "pending" if (a.get("interlocks") or {}).get("pending_release") else None}}
    clock = dict(info["clock"])
    if clock.get("source") not in ("CLOCK_MONOTONIC", "CLOCK_MONOTONIC_RAW", "CLOCK_BOOTTIME"):
        clock["source"] = "CLOCK_MONOTONIC"           # sim runs say so in their device id, not here
    clock["boot_id"] = clock.get("boot_id") or "unknown"
    calibration = {"top": info["calibration"]} if info.get("calibration") else {}
    msg = {"type": "hello", "wire": WIRE_VERSION, "seq": seq, "sdk": SDK_NAME, "kernel_sha256": info["kernel_sha256"],
           "rig": rig, "device": device, "clock": clock, "armed": bool(info["armed"]), "pack": info.get("pack"),
           "calibration": calibration, "arms": arms, "cameras": cameras or {}, "stop_sources": list(stop_sources),
           "latched": {"stopped": bool(info["latched"]["stopped"]), "link_lost": bool(info["latched"]["link_lost"]),
                       "faults": [code(f) for f in info["latched"]["faults"]]}}
    return _valid(msg)


def ack(local: dict, seq: int) -> dict:
    msg = {"type": "ack", "seq": seq, "ref": local["ref"], "sdk_t": max(0.0, float(local.get("sdk_t") or 0.0))}
    if local.get("duplicate"):
        msg["result"] = "duplicate"
    elif local.get("result") == "accepted":
        msg["result"] = "accepted"
        msg["tau0"] = 0.0
    else:
        msg["result"] = "refused"
        msg["reason"] = REASONS.get(local.get("reason"), "trajectory_invalid")
    if local.get("start_measured") and msg["result"] == "accepted":
        msg["start_measured"] = {arm: {"q": v["q"], "jaw": v.get("jaw"), "age_ms": max(0.0, float(v.get("age_ms") or 0.0))}
                                 for arm, v in local["start_measured"].items()}
    detail = {k: v for k, v in local.items() if k in ("reason", "message", "detail", "acceptance", "rest", "proximity")}
    if detail:
        msg["detail"] = _detail({"sdk": detail})
    return _valid(msg)


GATE_OUTCOMES = {"complete": "complete", "unsettled": "timeout", "stopped": "stopped", "fault": "failed",
                 "connector_error": "failed", "not_arrived": "failed", "verify_failed": "failed"}


def gate_report(local: dict, seq: int) -> dict:
    outcome = local.get("outcome", "")
    rest = {arm: {k: v for k, v in r.items() if v is not None} for arm, r in (local.get("rest") or {}).items()}
    msg = {"type": "gate_report", "seq": seq, "segment": local["segment"], "gate": local.get("gate", "hold"),
           "outcome": GATE_OUTCOMES.get(outcome, "held"),
           "evidence": _detail(dict(local.get("evidence") or {}, sdk_outcome=outcome)), "rest": rest,
           "observations": list(local.get("observations") or [])}
    return _valid(msg)


def stop_answer(local: dict, seq: int) -> dict:
    arms = {}
    for arm, a in (local.get("arms") or {}).items():
        # holding only on the owner's measured confirmation; an unpowered brakeless arm is not held
        row = {"holding": a.get("holding") is True}
        if a.get("time_to_hold_ms") is not None:
            row["time_to_hold_ms"] = float(a["time_to_hold_ms"])
        if a.get("q"):
            row["q"] = a["q"]
        text = a.get("console") or ("unpowered: nothing holds the arm; support it" if a.get("powered") is False else None)
        if text:
            row["detail"] = text[:2000]
        arms[arm] = row
    kind = "stop_not_confirmed" if local.get("not_confirmed") else "stopped"
    msg = {"type": "event", "seq": seq, "sdk_t": max(0.0, float(local.get("sdk_t") or 0.0)), "kind": kind,
           "ref": local.get("ref") or f"stp_local_{seq}", "arms": arms,
           "detail": _detail({"source": local.get("source"), "verified": local.get("verified"),
                              "elapsed_ms": local.get("elapsed_ms")})}
    return _valid(msg)


def event(local: dict, seq: int) -> dict | None:
    """One rt event in wire form, or None for local-only bookkeeping."""
    if local.get("type") == "gate_report":
        return gate_report(local, seq)
    kind = local.get("kind")
    if kind in LOCAL_ONLY:
        return None
    if kind == "stopped":
        return stop_answer(local, seq)
    wkind = EVENTS.get(kind, "refusal")
    msg = {"type": "event", "seq": seq, "sdk_t": max(0.0, float(local.get("sdk_t") or 0.0)), "kind": wkind}
    for key in ("segment", "id", "arm", "latched"):
        if local.get(key) is not None:
            msg[key] = local[key]
    if local.get("tau") is not None:
        msg["tau"] = max(0.0, float(local["tau"]))
    if wkind == "trigger":
        msg.setdefault("id", code(local.get("kind")))
        vals = {k: float(v) for k, v in (local.get("values") or {}).items() if isinstance(v, (int, float)) and math.isfinite(v)}
        if vals:
            msg["values"] = vals
        action = local.get("action")
        if isinstance(action, str):
            msg["action"] = code(action)
    if wkind in ("arm_offline", "fault"):
        msg["arm"] = local.get("arm") or "unknown"
        msg["cause"] = code(local.get("cause") or local.get("error") or kind)
    rest = {k: v for k, v in local.items() if k not in ("type", "seq", "rt_seq", "sdk_t", "kind", "segment", "id", "arm",
                                                         "latched", "tau", "values", "action", "cause")}
    msg["detail"] = _detail(dict(rest, sdk_kind=kind))
    return _valid(msg)


def observation(manifest: dict, seq: int) -> dict:
    cam = dict(manifest["camera"])
    intr = cam.pop("intrinsics", None) or {}
    if intr.get("width") and intr.get("height") and intr.get("fx") and intr.get("fy"):
        cam["intrinsics"] = {"w": int(intr["width"]), "h": int(intr["height"]), "fx": float(intr["fx"]),
                             "fy": float(intr["fy"]), **({"cx": float(intr["ppx"])} if intr.get("ppx") is not None else {}),
                             **({"cy": float(intr["ppy"])} if intr.get("ppy") is not None else {})}
    cam = {k: v for k, v in cam.items() if v is not None}
    br = {k: v for k, v in manifest["bracket"].items() if v is not None}
    msg = {"type": "observation", "wire": WIRE_VERSION, "seq": seq, "obs": manifest["obs"],
           "capture": manifest.get("capture"), "kind": manifest["kind"], "camera": cam,
           "t_exposure": manifest["t_exposure"], "bracket": br,
           "arms": {a: {k: v for k, v in s.items() if v is not None} for a, s in manifest["arms"].items()},
           "blobs": manifest["blobs"], "privacy": manifest["privacy"]}
    if msg["capture"] is None:
        msg.pop("capture")
    return _valid(msg)
