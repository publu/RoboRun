"""Segment messages: parse and structurally validate; acceptance lives in ``acceptance``.

Supported:
- ``kind: move`` with ``trajectory.format`` ``rows30.v1`` (time-indexed rows on executed time, dt = 1/30 s) or
  ``path-rows.v1`` (geometric rows; the player steps along them by ``player.step`` per 1/15 s, holding while the
  arm lags more than ``player.track_limit``).
- ``valid_if.after`` continuations, triggers on locally computable signals, end gates (hold / observe / verify).
  A ``verify`` gate's ``require`` is evaluated locally: one condition on a locally computed signal, stat ``value`` or
  ``delta_since_start``; anything else is refused (``unsupported_gate``) rather than reported as passed unchecked.

Refused with a typed reason (not emulated): ``jaw_only`` trajectories and ``close_until`` / ``release`` kinds
(contact behaviours stay in the robot runtime), branches,
and ``stream7d.v1`` player features that only the robot runtime's own player implements (``loaded``, ``cup``).
"""
from __future__ import annotations

import base64
import binascii
import math
from dataclasses import dataclass, field

import numpy as np

from .canonical import digest as content_digest
from .datatypes import Refused

SIG_KIND = "segment"
FORMATS = ("rows30.v1", "path-rows.v1")
KINDS = ("move",)
GATES = ("hold", "observe", "verify")
# Signals computed locally (manifest_wire.KNOWN_SIGNALS minus tool_speed and press, plus two SDK-local ones).
SIGNALS = ("jaw", "bias_j1_4", "bias_j5_6", "effort", "jaw_effort", "min_gap_mm", "tool_z", "tool_lag",
           "jaw_floor_gap", "track_err", "effort_max")
STATS = ("value", "range", "delta")
# verify-gate requirements evaluated locally when the arm has settled (runtime._gate)
VERIFY_SIGNALS = ("jaw", "tool_z", "bias_j1_4", "bias_j5_6", "effort", "jaw_effort", "effort_max", "track_err",
                  "tool_lag", "min_gap_mm")
VERIFY_STATS = ("value", "delta_since_start")
OPS = ("<", "<=", ">", ">=", "==", "!=")
MAX_ROWS = 4096


def _num(value, name, *, lo=-math.inf, hi=math.inf) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not lo <= value <= hi:
        raise Refused("malformed", f"{name} must be a finite number in [{lo}, {hi}]")
    return float(value)


def decode_rows(trajectory: dict, joints: int = 7) -> np.ndarray:
    """rows: little-endian float32 bytes (msgpack frames) or a list of lists (JSON debugging)."""
    shape = trajectory.get("shape")
    rows = trajectory.get("rows")
    if (not isinstance(shape, (list, tuple)) or len(shape) != 2 or any(isinstance(v, bool) or not isinstance(v, int) for v in shape)
            or not 1 <= shape[0] <= MAX_ROWS or shape[1] != joints):
        raise Refused("malformed", f"trajectory shape must be [n<= {MAX_ROWS}, {joints}]")
    if isinstance(rows, (bytes, bytearray, memoryview)):
        raw = bytes(rows)
        if len(raw) != shape[0] * shape[1] * 4:
            raise Refused("malformed", "rows byte length does not match the shape")
        arr = np.frombuffer(raw, dtype="<f4").astype(np.float64).reshape(shape)
    elif isinstance(rows, str):                    # the wire's JSON rendering: base64
        try:
            raw = base64.b64decode(rows, validate=True)
        except (binascii.Error, ValueError):
            raise Refused("malformed", "rows are not base64") from None
        if len(raw) != shape[0] * shape[1] * 4:
            raise Refused("malformed", "rows byte length does not match the shape")
        arr = np.frombuffer(raw, dtype="<f4").astype(np.float64).reshape(shape)
    elif isinstance(rows, list):
        try:
            arr = np.asarray(rows, dtype=np.float64)
        except (TypeError, ValueError):
            raise Refused("malformed", "rows must be numbers") from None
        if arr.shape != tuple(shape):
            raise Refused("malformed", "rows do not match the shape")
    else:
        raise Refused("malformed", "rows must be float32 bytes or a list")
    if not np.isfinite(arr).all():
        raise Refused("nonfinite", "trajectory rows contain non-finite values")
    return arr


def encode_rows(rows) -> dict:
    arr = np.asarray(rows, dtype="<f4")
    return {"shape": [int(arr.shape[0]), int(arr.shape[1])], "rows": arr.tobytes()}


@dataclass(frozen=True)
class Trigger:
    id: str
    when: dict
    do: object
    then: object


@dataclass(frozen=True)
class Segment:
    raw: dict = field(repr=False)
    digest: str
    id: str
    seq: int
    task: str
    generation: int
    index: int
    arm: str
    semantics: str
    kind: str
    label: str
    context: dict
    valid_if: dict
    format: str
    rows: np.ndarray = field(repr=False)
    dt: float
    player: dict
    limits: dict
    triggers: tuple
    complete: dict
    end_gate: dict
    on_link_loss: str
    after: dict | None

    @property
    def duration_s(self) -> float:
        return (len(self.rows) - 1) * self.dt if self.format == "rows30.v1" else math.nan

    def jaw_column(self) -> np.ndarray:
        return self.rows[:, -1]


def _check_condition(cond, ids: set[str]):
    if not isinstance(cond, dict):
        raise Refused("malformed", "trigger conditions are objects")
    if "any" in cond or "all" in cond:
        key = "any" if "any" in cond else "all"
        if not isinstance(cond[key], list) or not cond[key]:
            raise Refused("malformed", f"{key} needs a non-empty list")
        for c in cond[key]:
            _check_condition(c, ids)
        return
    if "ref" in cond:
        if cond["ref"] not in ids:
            raise Refused("malformed", f"trigger ref {cond['ref']!r} names no earlier trigger")
        return
    sig = cond.get("sig")
    if sig not in SIGNALS:
        raise Refused("unsupported_trigger_signal", f"signal {sig!r} is not computed locally in this SDK version",
                      supported=list(SIGNALS))
    if cond.get("op") not in OPS:
        raise Refused("malformed", "trigger op must be <, <=, >, >=, == or !=")
    _num(cond.get("v"), "trigger value")
    stat = cond.get("stat", "value")
    if stat not in STATS or (stat != "value" and sig != "jaw"):
        # a trigger that could never fire would fail silently: refuse it instead
        raise Refused("unsupported_trigger_stat", f"stat {stat!r} on {sig!r} is not computed locally in this SDK version")
    if stat != "value":
        _num(cond.get("w"), "trigger window", lo=0.0, hi=10.0)


def _check_require(req) -> None:
    """A verify gate is reported ``complete`` only after its requirement was evaluated here; a requirement this SDK
    cannot evaluate is refused, never passed unchecked."""
    if not isinstance(req, dict) or not set(req) <= {"sig", "stat", "op", "v"}:
        raise Refused("unsupported_gate", "verify gates evaluate one condition (sig, stat, op, v) in this SDK version; "
                      "any/all/ref and windowed stats are not evaluated locally")
    if req.get("sig") not in VERIFY_SIGNALS:
        raise Refused("unsupported_gate", f"verify signal {req.get('sig')!r} is not computed locally",
                      supported=list(VERIFY_SIGNALS))
    if req.get("stat", "value") not in VERIFY_STATS:
        raise Refused("unsupported_gate", f"verify stat {req.get('stat')!r} is not evaluated locally",
                      supported=list(VERIFY_STATS))
    if req.get("op") not in OPS:
        raise Refused("malformed", "require.op must be <, <=, >, >=, == or !=")
    _num(req.get("v"), "require.v")


def parse_segment(msg: dict) -> Segment:
    if not isinstance(msg, dict) or msg.get("type") != "segment":
        raise Refused("malformed", "not a segment message")
    def text(key, src=msg):
        v = src.get(key)
        if not isinstance(v, str) or not v or len(v) > 200:
            raise Refused("malformed", f"{key} must be a short string")
        return v
    def integer(key, src=msg):
        v = src.get(key)
        if isinstance(v, bool) or not isinstance(v, int) or v < 0:
            raise Refused("malformed", f"{key} must be a non-negative integer")
        return v
    kind = text("kind")
    if kind not in KINDS:
        raise Refused("unsupported_kind", f"kind {kind!r} is not played by this SDK version (move only; "
                      "close/release are played by the robot runtime)")
    if msg.get("branches"):
        raise Refused("unsupported_branches", "pre-sent branches are not supported by this SDK version")
    if msg.get("scale"):
        raise Refused("unsupported_format", "speed-scale rules are not supported by this SDK version; ignoring them would play faster than planned")
    traj = msg.get("trajectory")
    if not isinstance(traj, dict):
        raise Refused("malformed", "trajectory missing")
    fmt = traj.get("format")
    if fmt not in FORMATS:
        raise Refused("unsupported_format", f"trajectory format {fmt!r} unsupported", supported=list(FORMATS))
    rows = decode_rows(traj)
    if fmt == "rows30.v1":
        dt = _num(traj.get("dt", 1 / 30), "dt", lo=1 / 31, hi=1 / 29)
        if len(rows) < 2:
            raise Refused("malformed", "rows30.v1 needs at least two rows")
    else:
        dt = 1 / 30
    player = dict(msg.get("player") or {})
    if fmt == "path-rows.v1":
        profile = player.get("profile", "geometric.v1")
        if profile not in ("geometric.v1", "stream7d.v1"):
            raise Refused("unsupported_player", f"player profile {profile!r} unsupported")
        if profile == "stream7d.v1" and (player.get("loaded") or player.get("cup") or player.get("mode", "move") != "move"):
            raise Refused("unsupported_player_feature", "stream7d.v1 loaded/cup/non-move behaviours need the robot runtime's "
                          "own player; this player only steps free unloaded paths")
        player["step"] = _num(player.get("step", 0.034), "player.step", lo=1e-4, hi=0.2)
        player["track_limit"] = _num(player.get("track_limit", 0.085), "player.track_limit", lo=1e-3, hi=0.5)
        player["window_s"] = _num(player.get("window_s", 12.0), "player.window_s", lo=0.1, hi=120.0)
        player["profile"] = profile
    valid_if = msg.get("valid_if")
    if not isinstance(valid_if, dict):
        raise Refused("malformed", "valid_if missing")
    start_q = valid_if.get("start_q")
    if start_q is not None:
        if not isinstance(start_q, list) or len(start_q) != rows.shape[1] - 1:
            raise Refused("malformed", "valid_if.start_q must list every joint")
        for v in start_q:
            _num(v, "start_q")
    after = valid_if.get("after")
    if after is not None and (not isinstance(after, dict) or not isinstance(after.get("segment"), str)):
        raise Refused("malformed", "valid_if.after names a segment")
    if start_q is None and after is None:
        raise Refused("malformed", "a segment needs start_q or an after-continuation")
    _num(valid_if.get("tol_rad", 0.01), "tol_rad", lo=0.0, hi=0.1)
    _num(valid_if.get("expires_after_s", 10), "expires_after_s", lo=0.0, hi=600.0)
    # jaw_tol and the rest test are bounded by the active pack at acceptance (``loosening_limits``); here: shape only
    _num(valid_if.get("jaw_tol", 0.005), "jaw_tol", lo=0.0)
    if valid_if.get("jaw") is not None:
        _num(valid_if["jaw"], "valid_if.jaw")
    rest = valid_if.get("rest")
    if rest is not None:
        if not isinstance(rest, dict) or set(rest) - {"window_s", "range_rad", "jaw_range"}:
            raise Refused("malformed", "valid_if.rest has window_s, range_rad and optionally jaw_range")
        for key in ("window_s", "range_rad", "jaw_range"):
            if rest.get(key) is not None:
                _num(rest[key], f"valid_if.rest.{key}", lo=0.0)
    triggers = []
    ids: set[str] = set()
    for t in msg.get("triggers") or ():
        if not isinstance(t, dict) or not isinstance(t.get("id"), str):
            raise Refused("malformed", "triggers need ids")
        _check_condition(t.get("when"), ids)
        do = t.get("do")
        if do not in (None, "hold", "report") and not (isinstance(do, dict) and set(do) == {"pause_s"}):
            raise Refused("unsupported_trigger_action", f"trigger action {do!r} unsupported (hold, report, pause_s; "
                          "jaw_set belongs to closes)")
        if isinstance(do, dict):
            _num(do["pause_s"], "pause_s", lo=0.0, hi=1.5)
        then = t.get("then") or "end"
        if then not in ("end", "report", "hold", "continue"):
            raise Refused("unsupported_trigger_then", f"trigger then {then!r} unsupported (branches are not supported by this SDK version)")
        ids.add(t["id"])
        triggers.append(Trigger(t["id"], t["when"], do, then))
    gate = dict(msg.get("end_gate") or {"kind": "hold"})
    if gate.get("kind") not in GATES:
        raise Refused("malformed", f"end_gate.kind must be one of {GATES}")
    if gate["kind"] == "verify":
        _check_require(gate.get("require"))
    complete = dict(msg.get("complete") or {})
    complete["still_s"] = _num(complete.get("still_s", 0.5), "complete.still_s", lo=0.0, hi=10.0)
    complete["tol_rad"] = _num(complete.get("tol_rad", 0.003), "complete.tol_rad", lo=0.0, hi=0.1)
    complete["timeout_s"] = _num(complete.get("timeout_s", 3.0), "complete.timeout_s", lo=0.1, hi=60.0)
    on_loss = msg.get("on_link_loss", "hold")
    if on_loss not in ("hold", "brake"):
        raise Refused("malformed", "on_link_loss is hold or brake")
    limits = dict(msg.get("limits") or {})
    for key, value in limits.items():
        _num(value, f"limits.{key}", lo=0.0)
    context = msg.get("context") or {}
    if not isinstance(context, dict):
        raise Refused("malformed", "context must be an object")
    return Segment(raw=msg, digest=content_digest(msg), id=text("id"), seq=integer("seq"),
                   task=str(msg.get("task", "")), generation=integer("generation"), index=integer("index"),
                   arm=text("arm"), semantics=text("semantics"), kind=kind, label=str(msg.get("label", "")),
                   context=context, valid_if=valid_if, format=fmt, rows=rows, dt=dt, player=player, limits=limits,
                   triggers=tuple(triggers), complete=complete, end_gate=gate, on_link_loss=on_loss, after=after)
