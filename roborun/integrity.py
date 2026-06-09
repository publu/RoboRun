"""Tamper-evident run integrity — hash chain + SHA-256 Merkle tree
+ optional Ed25519 signature.

A "run" is a directory:
    run.jsonl       ordered event timeline (one JSON object per line);
                    events written by roborun carry `prev` = SHA-256 of the
                    previous event, so the log is tamper-evident while being
                    written, not only after sealing
    run.seal        merkle root, per-event hashes, timestamp, signature
    manifest.json   run id, start time, link to the previous sealed run

Seal: hash each event (canonical JSON), build a binary Merkle tree,
sign the root. Verify: recompute everything — per-event hashes, chain
continuity, Merkle root, signature — and report the exact event that
fails. Same primitives as Git and Certificate Transparency.

Stdlib-only core; Ed25519 signing activates if `cryptography` is
installed ([crypto] extra). The merkle root is small enough to share
anywhere — anyone holding a copy of it can later prove the run wasn't
quietly resealed.

What this proves: the recorded timeline has not been altered since sealing.
What it does not prove: that the robot's sensors observed reality correctly.

CLI:
    python -m roborun.integrity seal   <run_dir>
    python -m roborun.integrity verify <run_dir>
    python -m roborun.integrity tamper <run_dir> [--event N]
"""
from __future__ import annotations

import hashlib
import json
import time
from pathlib import Path
from typing import Any

SEAL_VERSION = 2
GENESIS = "0" * 64
_KEY_DIR = Path.home() / ".roborun"
_KEY_PATH = _KEY_DIR / "ed25519.key"


def canonical(event: dict[str, Any]) -> bytes:
    return json.dumps(event, sort_keys=True, separators=(",", ":"), default=str).encode()


def hash_event(event: dict[str, Any]) -> str:
    return hashlib.sha256(canonical(event)).hexdigest()


def merkle_root(hashes: list[str]) -> str:
    """Binary Merkle tree over hex leaf hashes; odd nodes promote."""
    if not hashes:
        return hashlib.sha256(b"").hexdigest()
    level = [bytes.fromhex(h) for h in hashes]
    while len(level) > 1:
        nxt = []
        for i in range(0, len(level) - 1, 2):
            nxt.append(hashlib.sha256(level[i] + level[i + 1]).digest())
        if len(level) % 2:
            nxt.append(level[-1])
        level = nxt
    return level[0].hex()


def _load_events(run_dir: Path) -> list[str]:
    """Raw JSONL lines — hashing operates on parsed canonical form."""
    path = run_dir / "run.jsonl"
    if not path.exists():
        raise FileNotFoundError(f"no run.jsonl in {run_dir}")
    return [ln for ln in path.read_text().splitlines() if ln.strip()]


def _hash_lines(lines: list[str]) -> list[str]:
    return [hash_event(json.loads(ln)) for ln in lines]


def _sign(root_hex: str, count: int, sealed_at: str) -> dict[str, str] | None:
    try:
        from cryptography.hazmat.primitives.asymmetric.ed25519 import (
            Ed25519PrivateKey,
        )
        from cryptography.hazmat.primitives import serialization
    except ImportError:
        return None
    _KEY_DIR.mkdir(parents=True, exist_ok=True)
    if _KEY_PATH.exists():
        key = serialization.load_pem_private_key(_KEY_PATH.read_bytes(), password=None)
    else:
        key = Ed25519PrivateKey.generate()
        _KEY_PATH.write_bytes(key.private_bytes(
            serialization.Encoding.PEM,
            serialization.PrivateFormat.PKCS8,
            serialization.NoEncryption()))
        _KEY_PATH.chmod(0o600)
    message = f"{root_hex}|{count}|{sealed_at}".encode()
    sig = key.sign(message)
    pub = key.public_key().public_bytes(
        serialization.Encoding.Raw, serialization.PublicFormat.Raw)
    return {"algo": "ed25519", "signature": sig.hex(), "public_key": pub.hex()}


def _check_signature(seal: dict) -> bool | None:
    """True/False if checkable, None if unsigned or cryptography missing."""
    sig = seal.get("signature")
    if not sig:
        return None
    try:
        from cryptography.hazmat.primitives.asymmetric.ed25519 import (
            Ed25519PublicKey,
        )
    except ImportError:
        return None
    message = f"{seal['merkle_root']}|{seal['event_count']}|{seal['sealed_at']}".encode()
    try:
        Ed25519PublicKey.from_public_bytes(
            bytes.fromhex(sig["public_key"])).verify(
            bytes.fromhex(sig["signature"]), message)
        return True
    except Exception:
        return False


def seal_run(run_dir: str | Path) -> dict[str, Any]:
    run_dir = Path(run_dir)
    lines = _load_events(run_dir)
    hashes = _hash_lines(lines)
    root = merkle_root(hashes)
    sealed_at = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    seal = {
        "version": SEAL_VERSION,
        "algo": "sha256-merkle",
        "event_count": len(lines),
        "merkle_root": root,
        "sealed_at": sealed_at,
        "event_hashes": hashes,
        "signature": _sign(root, len(lines), sealed_at),
    }
    (run_dir / "run.seal").write_text(json.dumps(seal, indent=1))
    return {"ok": True, "merkle_root": root, "event_count": len(lines),
            "sealed_at": sealed_at, "signed": seal["signature"] is not None}


def verify_chain(lines: list[str]) -> dict[str, Any]:
    """Check `prev`-hash continuity. Legacy runs without `prev` are skipped."""
    if not lines:
        return {"chain_checked": False}
    events = [json.loads(ln) for ln in lines]
    if "prev" not in events[0]:
        return {"chain_checked": False}
    prev = GENESIS
    for i, evt in enumerate(events):
        if evt.get("prev") != prev:
            return {"chain_checked": True, "chain_intact": False, "chain_break": i,
                    "reason": f"event {i:04d} chain break — prev hash does not match event {i-1:04d}"}
        prev = hash_event(evt)
    return {"chain_checked": True, "chain_intact": True, "chain_head": prev}


def verify_run(run_dir: str | Path) -> dict[str, Any]:
    run_dir = Path(run_dir)
    try:
        lines = _load_events(run_dir)
    except FileNotFoundError as exc:
        return {"ok": False, "verified": False, "reason": str(exc)}

    chain = verify_chain(lines)

    seal_path = run_dir / "run.seal"
    if not seal_path.exists():
        # Unsealed (live or crashed) run: the chain is still checkable.
        result = {"ok": True, "verified": False, **chain,
                  "reason": "not sealed — chain checked only"}
        if chain.get("chain_intact"):
            result["reason"] = f"not sealed — chain intact through {len(lines)} events"
        elif chain.get("chain_checked"):
            result["reason"] = chain["reason"]
        return result

    seal = json.loads(seal_path.read_text())

    if len(lines) != seal["event_count"]:
        return {"ok": True, "verified": False, **chain,
                "reason": f"event count mismatch — sealed {seal['event_count']}, found {len(lines)}",
                "merkle_root": seal["merkle_root"]}

    hashes = _hash_lines(lines)
    for i, (h, expected) in enumerate(zip(hashes, seal["event_hashes"])):
        if h != expected:
            return {"ok": True, "verified": False, "failed_event": i, **chain,
                    "reason": f"event {i:04d} hash mismatch",
                    "expected": expected, "found": h,
                    "merkle_root": seal["merkle_root"]}

    root = merkle_root(hashes)
    if root != seal["merkle_root"]:
        return {"ok": True, "verified": False, **chain,
                "reason": "merkle root mismatch — event hashes reordered",
                "merkle_root": seal["merkle_root"]}

    if chain.get("chain_checked") and not chain.get("chain_intact"):
        return {"ok": True, "verified": False, **chain,
                "reason": chain["reason"], "merkle_root": root}

    return {"ok": True, "verified": True, "event_count": len(lines),
            "merkle_root": root, "sealed_at": seal["sealed_at"],
            "signature_valid": _check_signature(seal),
            **chain}


def tamper_run(run_dir: str | Path, event_index: int | None = None) -> dict[str, Any]:
    """Demo helper: flip one byte of one event so verification fails."""
    run_dir = Path(run_dir)
    lines = _load_events(run_dir)
    if not lines:
        return {"ok": False, "error": "run is empty"}
    idx = event_index if event_index is not None else min(42, len(lines) - 1)
    idx = max(0, min(idx, len(lines) - 1))
    event = json.loads(lines[idx])
    ts = event.get("ts")
    if isinstance(ts, (int, float)):
        event["ts"] = ts + 0.000001
    else:
        event["title"] = str(event.get("title", "")) + " "
    lines[idx] = json.dumps(event, default=str)
    (run_dir / "run.jsonl").write_text("\n".join(lines) + "\n")
    return {"ok": True, "tampered_event": idx,
            "note": "one value changed by the smallest representable amount"}


def snapshot_run(events: list[dict], run_dir: str | Path,
                 manifest: dict[str, Any] | None = None) -> dict[str, Any]:
    """Write an event list as a run directory (no seal)."""
    run_dir = Path(run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)
    with open(run_dir / "run.jsonl", "w") as f:
        for evt in events:
            f.write(json.dumps(evt, default=str) + "\n")
    (run_dir / "manifest.json").write_text(json.dumps({
        "created_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "event_count": len(events),
        **(manifest or {}),
    }, indent=2))
    return {"ok": True, "run_dir": str(run_dir), "event_count": len(events)}


def _cli() -> int:
    import argparse
    p = argparse.ArgumentParser(prog="roborun.integrity",
                                description="Seal and verify robot runs")
    p.add_argument("command", choices=["seal", "verify", "tamper"])
    p.add_argument("run_dir")
    p.add_argument("--event", type=int, default=None)
    args = p.parse_args()
    if args.command == "seal":
        r = seal_run(args.run_dir)
        print(f"SEALED — {r['event_count']} events")
        print(f"merkle root: {r['merkle_root']}")
        print(f"signed: {'ed25519' if r['signed'] else 'no (pip install cryptography)'}")
        return 0
    if args.command == "tamper":
        r = tamper_run(args.run_dir, args.event)
        if r["ok"]:
            print(f"tampered event {r['tampered_event']:04d} — {r['note']}")
        return 0
    r = verify_run(args.run_dir)
    if r.get("verified"):
        sig = r.get("signature_valid")
        sig_note = {True: "signature valid", False: "SIGNATURE INVALID", None: "unsigned"}[sig]
        chain_note = "chain intact" if r.get("chain_intact") else "no chain (legacy run)"
        print(f"VERIFIED — {r['event_count']} events · {chain_note} · {sig_note}")
        print(f"merkle root: {r['merkle_root']}")
        return 0
    print(f"FAILED — {r['reason']}")
    if "expected" in r:
        print(f"expected: sha256:{r['expected'][:16]}…")
        print(f"found:    sha256:{r['found'][:16]}…")
    return 1


if __name__ == "__main__":
    raise SystemExit(_cli())
