"""Development helpers: DEV-ONLY keys, the bundled sim pack, and a segment builder for tests and examples.

The dev keys come from fixed public seeds. Anything signed with them is test data: ``roborun run --live`` refuses
dev-signed packs unless ``--dev-keys-ok`` is given explicitly. Real packs are signed by the API's root key.
"""
from __future__ import annotations

import copy
import hashlib
import json
from importlib import resources

import numpy as np

from .rt.canonical import from_jsonable, to_jsonable
from .rt.pack import SIG_KIND as PACK_KIND
from .rt.pack import Pack, parse_pack
from .rt.segment import SIG_KIND as SEGMENT_KIND
from .rt.segment import encode_rows
from .rt.signing import SigningKey

DEV_ROOT = SigningKey("dev-root-NOT-FOR-ROBOTS", hashlib.sha256(b"manifest-sdk dev root key - NOT FOR ROBOTS").digest())
DEV_SEGMENTS = SigningKey("dev-seg-NOT-FOR-ROBOTS", hashlib.sha256(b"manifest-sdk dev segment key - NOT FOR ROBOTS").digest())
DEV_KIDS = (DEV_ROOT.kid, DEV_SEGMENTS.kid)


def dev_root_keys() -> dict:
    return {DEV_ROOT.kid: DEV_ROOT.public}


def sim_pack_raw() -> dict:
    text = resources.files("roborun_sdk").joinpath("data/sim-yam-metal.pack.json").read_text()
    return from_jsonable(json.loads(text))


def sim_pack() -> Pack:
    raw = sim_pack_raw()
    return parse_pack(raw, signed=True)


def sign_pack(raw: dict, key: SigningKey = DEV_ROOT) -> dict:
    return key.sign(raw, PACK_KIND)


STREAM7D_MOVE = {"profile": "stream7d.v1", "mode": "move", "step": 0.034, "jaw_step": 0.04, "track_limit": 0.085,
                 "loaded": False, "window_s": 12.0, "empty_margin": 0.03, "cup": False}


def make_segment(rows, *, arm: str = "left", semantics: str, pack_digest: str, seq: int = 1, sid: str = "seg_1",
                 task: str = "tsk_sim", generation: int = 1, index: int = 0, fmt: str = "rows30.v1",
                 start_q=None, jaw: float | None = None, label: str = "move", limits: dict | None = None,
                 player: dict | None = None, triggers: list | None = None, end_gate: dict | None = None,
                 plan_basis_obs: str = "obs_sim", calibration: str = "cal_sim", after: dict | None = None,
                 complete: dict | None = None, key: SigningKey | None = DEV_SEGMENTS, extra: dict | None = None) -> dict:
    """A manifest.wire.v1 segment (validates against manifest_wire.Segment), signed with the DEV segment key."""
    rows = np.asarray(rows, float)
    traj = {"format": fmt, **encode_rows(rows)}
    if fmt == "rows30.v1":
        traj["dt"] = 1 / 30
    valid_if = {"start_q": [float(v) for v in (rows[0, :-1] if start_q is None else start_q)], "tol_rad": 0.01,
                "rest": {"window_s": 0.3, "range_rad": 0.0015},
                "jaw": float(rows[0, -1] if jaw is None else jaw), "jaw_tol": 0.005, "expires_after_s": 10}
    if after is not None:
        valid_if["after"] = after
    msg = {"type": "segment", "wire": 1, "seq": seq, "id": sid, "task": task, "generation": generation, "index": index,
           "arm": arm, "semantics": semantics,
           "context": {"pack": pack_digest, "calibration": calibration, "plan_basis_obs": plan_basis_obs},
           "valid_if": valid_if, "kind": "move", "label": label, "trajectory": traj,
           "limits": dict(limits or {"ee_speed_m_s": 0.04}), "end_gate": dict(end_gate or {"kind": "hold"}),
           "on_link_loss": "hold"}
    if fmt == "path-rows.v1":
        msg["player"] = dict(player or STREAM7D_MOVE)
    elif player is not None:
        msg["player"] = player
    if triggers:
        msg["triggers"] = triggers
    if complete:
        msg["complete"] = complete
    if extra:
        msg.update(copy.deepcopy(extra))
    return key.sign(msg, SEGMENT_KIND) if key is not None else msg


def dump_json(msg: dict) -> str:
    return json.dumps(to_jsonable(msg), indent=1)


def selftest_segment(pack: Pack, arm: str, start_q, jaw: float, *, delta_rad: float = 0.03, seconds: float = 1.0,
                     key: SigningKey = DEV_SEGMENTS, measured_jaw: float | None = None) -> dict:
    """A short smooth joint-1 move from the arm's measured pose (doctor and tests). ``jaw`` is held in every row (a
    planner holds the jaw at its present command); ``measured_jaw`` (default ``jaw``) is the start predicate."""
    n = int(round(seconds * 30)) + 1
    s = np.linspace(0.0, 1.0, n)
    ease = s * s * (3 - 2 * s)
    rows = np.tile(np.r_[np.asarray(start_q, float), jaw], (n, 1))
    rows[:, 0] += delta_rad * ease
    ap = pack.arm(arm)
    return make_segment(rows, arm=arm, semantics=ap.semantics, pack_digest=pack.digest, sid="seg_selftest",
                        jaw=jaw if measured_jaw is None else measured_jaw, label="selftest",
                        limits={"ee_speed_m_s": 0.10}, key=key)
