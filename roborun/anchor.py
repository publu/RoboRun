"""External timestamp anchoring for run seals — OpenTimestamps.

A seal's Merkle root proves internal consistency; anchoring proves the root
existed at a moment an *external* clock witnessed. OpenTimestamps commits the
digest into the Bitcoin blockchain via public calendar servers: submission is
instant (the calendar returns a pending attestation), the Bitcoin attestation
becomes available after the calendar's next commitment tx confirms (hours).
`upgrade` swaps pending attestations for Bitcoin ones when they are ready.

Everything here is best-effort and optional: no `opentimestamps` package or
no network simply yields status "unanchored". The proof is a standard
detached `.ots` file, verifiable with any OTS client.

Status vocabulary (consumed by recorder.verify_mcap_run and the flight deck):
    anchored   — at least one Bitcoin attestation (block height recorded)
    pending    — calendars accepted the digest, Bitcoin attestation not yet
    unanchored — no .ots / library missing / submission failed
"""
from __future__ import annotations

import hashlib
import time
from pathlib import Path
from typing import Any

CALENDARS = (
    "https://a.pool.opentimestamps.org",
    "https://b.pool.opentimestamps.org",
    "https://alice.btc.calendar.opentimestamps.org",
    "https://bob.btc.calendar.opentimestamps.org",
)

_SUBMIT_TIMEOUT = 10.0  # per calendar; sealing must not hang a robot


def _ots():
    """Import the opentimestamps modules, or None if not installed."""
    try:
        from opentimestamps.calendar import RemoteCalendar
        from opentimestamps.core.timestamp import Timestamp, DetachedTimestampFile
        from opentimestamps.core.op import OpSHA256
        from opentimestamps.core.notary import (
            PendingAttestation, BitcoinBlockHeaderAttestation,
        )
        from opentimestamps.core.serialize import (
            BytesSerializationContext, BytesDeserializationContext,
        )
        return {
            "RemoteCalendar": RemoteCalendar,
            "Timestamp": Timestamp,
            "DetachedTimestampFile": DetachedTimestampFile,
            "OpSHA256": OpSHA256,
            "PendingAttestation": PendingAttestation,
            "BitcoinBlockHeaderAttestation": BitcoinBlockHeaderAttestation,
            "BytesSerializationContext": BytesSerializationContext,
            "BytesDeserializationContext": BytesDeserializationContext,
        }
    except ImportError:
        return None


def available() -> bool:
    return _ots() is not None


def stamp_digest(digest: bytes, calendars: tuple[str, ...] = CALENDARS,
                 min_responses: int = 1) -> bytes | None:
    """Submit a 32-byte SHA-256 digest to OTS calendars.

    Returns serialized detached-timestamp (.ots) bytes with pending
    attestations, or None when the library is missing or every calendar
    submission failed (offline robot — anchor opportunistically later).
    """
    mods = _ots()
    if mods is None or len(digest) != 32:
        return None
    timestamp = mods["Timestamp"](digest)
    responses = 0
    for url in calendars:
        try:
            cal = mods["RemoteCalendar"](url)
            timestamp.merge(cal.submit(digest, timeout=_SUBMIT_TIMEOUT))
            responses += 1
        except Exception:
            continue
    if responses < min_responses:
        return None
    detached = mods["DetachedTimestampFile"](mods["OpSHA256"](), timestamp)
    ctx = mods["BytesSerializationContext"]()
    detached.serialize(ctx)
    return ctx.getbytes()


def stamp_file(path: str | Path, ots_path: str | Path | None = None) -> dict[str, Any]:
    """Stamp a file's SHA-256; write `<path>.ots` next to it on success."""
    path = Path(path)
    digest = hashlib.sha256(path.read_bytes()).digest()
    ots_bytes = stamp_digest(digest)
    out = Path(ots_path) if ots_path else path.with_suffix(path.suffix + ".ots")
    if ots_bytes is None:
        return {"status": "unanchored", "digest": digest.hex(),
                "reason": "opentimestamps unavailable or all calendars unreachable"}
    out.write_bytes(ots_bytes)
    return {"status": "pending", "digest": digest.hex(), "ots": str(out),
            "stamped_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}


def _walk_attestations(timestamp) -> list[tuple[Any, Any]]:
    """All (msg, attestation) pairs reachable from a Timestamp."""
    return list(timestamp.all_attestations())


def status(ots_path: str | Path, expected_digest: bytes | None = None) -> dict[str, Any]:
    """Inspect an .ots file: anchored / pending / unanchored (+ details).

    Verifies digest binding when expected_digest is given. Does not contact
    the network; pair with `upgrade()` to pull completed Bitcoin attestations.
    """
    mods = _ots()
    path = Path(ots_path)
    if not path.exists():
        return {"status": "unanchored", "reason": "no .ots file"}
    if mods is None:
        return {"status": "pending", "reason": "opentimestamps not installed; cannot inspect",
                "ots": str(path)}
    try:
        ctx = mods["BytesDeserializationContext"](path.read_bytes())
        detached = mods["DetachedTimestampFile"].deserialize(ctx)
    except Exception as exc:
        return {"status": "unanchored", "reason": f"corrupt .ots: {exc}"}
    if expected_digest is not None and detached.file_digest != expected_digest:
        return {"status": "unanchored",
                "reason": "ots digest does not match the seal: proof is for something else"}
    bitcoin, pending = [], []
    for _msg, att in _walk_attestations(detached.timestamp):
        if isinstance(att, mods["BitcoinBlockHeaderAttestation"]):
            bitcoin.append(att.height)
        elif isinstance(att, mods["PendingAttestation"]):
            pending.append(att.uri)
    if bitcoin:
        return {"status": "anchored", "bitcoin_blocks": sorted(bitcoin), "ots": str(path)}
    if pending:
        return {"status": "pending", "calendars": pending, "ots": str(path)}
    return {"status": "unanchored", "reason": "ots contains no attestations"}


def upgrade(ots_path: str | Path) -> dict[str, Any]:
    """Ask the calendars whether pending attestations are now Bitcoin-anchored.

    Rewrites the .ots in place when an upgrade lands. This is the
    "anchor opportunistically when connectivity returns" path for offline runs.
    """
    mods = _ots()
    path = Path(ots_path)
    if mods is None or not path.exists():
        return status(ots_path)
    try:
        ctx = mods["BytesDeserializationContext"](path.read_bytes())
        detached = mods["DetachedTimestampFile"].deserialize(ctx)
    except Exception as exc:
        return {"status": "unanchored", "reason": f"corrupt .ots: {exc}"}

    upgraded = False
    for msg, att in list(detached.timestamp.all_attestations()):
        if not isinstance(att, mods["PendingAttestation"]):
            continue
        try:
            cal = mods["RemoteCalendar"](att.uri)
            update = cal.get_timestamp(msg, timeout=_SUBMIT_TIMEOUT)
        except Exception:
            continue
        try:
            _merge_into(detached.timestamp, msg, update)
            upgraded = True
        except Exception:
            continue
    if upgraded:
        out_ctx = mods["BytesSerializationContext"]()
        detached.serialize(out_ctx)
        path.write_bytes(out_ctx.getbytes())
    return status(ots_path)


def _merge_into(timestamp, msg: bytes, update) -> None:
    """Merge `update` into the sub-timestamp of `timestamp` whose message is msg."""
    if timestamp.msg == msg:
        timestamp.merge(update)
        return
    for op, stamp in timestamp.ops.items():
        try:
            _merge_into(stamp, msg, update)
            return
        except ValueError:
            continue
    raise ValueError("message not found in timestamp tree")
