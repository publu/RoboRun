"""Cross-robot beacons — signed shared awareness over R2, no broker (spec §3.3b/c).

A beacon is a tiny signed JSON file: "I (robot A) saw <label> at <pose> at
<ts>, evidence in run <run_id>". Robots write to beacons/<robot_id>/<ts>.json
and poll the shared prefix on an interval. No images travel — a robot that
cares pulls the full frame from the referenced MCAP in R2 on demand.

Trust: each beacon is signed with the robot's Ed25519 identity (the same key
that signs run seals), so a cross-robot claim is verifiable and traceable to
a sealed, anchored source run. The black box becomes the fleet's trust layer.

Backends: R2 when configured (R2Store.from_env), else a shared local
directory (ROBORUN_BEACON_DIR) — useful for tests and NFS-style mounts.
Latency is seconds, by design; this is shared memory, not a control loop.
"""
from __future__ import annotations

import json
import os
import time
import uuid
from pathlib import Path
from typing import Any

from roborun.integrity import sign_message, verify_message

BEACON_TTL = 600.0  # poll window: beacons older than this are stale


# Keys added after signing (transport/verification metadata, not the claim).
_ENVELOPE_KEYS = ("signature", "published", "note", "signature_valid")


def _payload_bytes(beacon: dict) -> bytes:
    unsigned = {k: v for k, v in beacon.items() if k not in _ENVELOPE_KEYS}
    return json.dumps(unsigned, sort_keys=True, separators=(",", ":"),
                      default=str).encode()


def make_beacon(robot_id: str, label: str,
                x: float | None = None, y: float | None = None,
                z: float | None = None,
                run_id: str | None = None,
                frame_topic: str | None = None,
                frame_log_time: int | None = None,
                embedding_sha256: str | None = None,
                detail: dict | None = None) -> dict[str, Any]:
    beacon: dict[str, Any] = {
        "format": "roborun-beacon-v1",
        "beacon_id": uuid.uuid4().hex[:12],
        "robot_id": robot_id,
        "ts": time.time(),
        "label": label,
        "pose": {"x": x, "y": y, "z": z},
        "run_ref": {"run_id": run_id, "frame_topic": frame_topic,
                    "frame_log_time": frame_log_time},
        "embedding_sha256": embedding_sha256,
        "detail": detail or {},
    }
    beacon["signature"] = sign_message(_payload_bytes(beacon))
    return beacon


def verify_beacon(beacon: dict) -> bool | None:
    """True/False if checkable, None when unsigned or cryptography missing."""
    return verify_message(beacon.get("signature"), _payload_bytes(beacon))


# ── backends ──────────────────────────────────────────────────────────────

def _local_dir() -> Path | None:
    d = os.environ.get("ROBORUN_BEACON_DIR")
    return Path(d) if d else None


def emit_beacon(robot_id: str, label: str, **kwargs) -> dict[str, Any]:
    """Sign and publish a beacon to the shared prefix. Returns the beacon."""
    beacon = make_beacon(robot_id, label, **kwargs)
    key = f"beacons/{robot_id}/{int(beacon['ts'] * 1000)}.json"

    local = _local_dir()
    if local is not None:
        path = local / key
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(beacon, separators=(",", ":")))
        return {**beacon, "published": str(path)}

    try:
        from roborun.r2sync import R2Store
        r2 = R2Store.from_env()
    except Exception:
        r2 = None
    if r2 is None:
        return {**beacon, "published": None,
                "note": "no beacon backend (set ROBORUN_R2_BUCKET or ROBORUN_BEACON_DIR)"}
    r2.put_json(key, beacon)
    return {**beacon, "published": f"r2://{r2.bucket}/{key}"}


def poll_beacons(since: float | None = None,
                 exclude_robot: str | None = None,
                 verify: bool = True) -> list[dict[str, Any]]:
    """Read recent beacons from the shared prefix, signature-checked.

    Unverifiable beacons (bad signature) are dropped; unsigned ones are
    kept but flagged, so a fleet without `cryptography` still coordinates.
    """
    since = since if since is not None else time.time() - BEACON_TTL
    raw: list[dict] = []

    local = _local_dir()
    if local is not None and (local / "beacons").exists():
        for p in sorted((local / "beacons").glob("*/*.json")):
            try:
                raw.append(json.loads(p.read_text()))
            except Exception:
                continue
    else:
        try:
            from roborun.r2sync import R2Store
            r2 = R2Store.from_env()
        except Exception:
            r2 = None
        if r2 is None:
            return []
        for key in r2.list_keys("beacons/"):
            # key format: beacons/<robot>/<ms>.json — cheap time filter pre-fetch
            try:
                ms = int(Path(key).stem)
                if ms / 1000.0 < since:
                    continue
            except ValueError:
                pass
            obj = r2.get_json(key)
            if obj:
                raw.append(obj)

    out = []
    for b in raw:
        if b.get("ts", 0) < since:
            continue
        if exclude_robot and b.get("robot_id") == exclude_robot:
            continue
        sig = verify_beacon(b) if verify else None
        if sig is False:
            continue  # forged or corrupted: never surface it
        b["signature_valid"] = sig
        out.append(b)
    out.sort(key=lambda b: b.get("ts", 0), reverse=True)
    return out
