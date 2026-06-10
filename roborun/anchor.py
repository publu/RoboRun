"""External timestamp anchoring for run seals — RFC 3161 trusted timestamps.

A seal's Merkle root proves internal consistency; anchoring proves the root
existed at a moment an *external* clock witnessed. We submit the digest to a
public RFC 3161 Time-Stamp Authority — the same mechanism behind code
signing — and store the DER TimeStampResp as a detached `.tsr` file. The
proof is a standard token any RFC 3161 tool can inspect and verify:

    openssl ts -reply -in run.seal.tsr -text
    openssl ts -verify -digest <merkle_root> -in run.seal.tsr -CAfile ...

Anchoring is synchronous: the TSA answers in under a second, so a sealed run
is anchored the moment it closes. Everything here is best-effort and
optional: no `asn1crypto` package or no network simply yields status
"unanchored", and `upgrade()` re-stamps later (the offline-robot path).

Status vocabulary (consumed by recorder.verify_mcap_run and the flight deck):
    anchored   — a TSA granted a timestamp over this digest (time recorded)
    unanchored — no .tsr / library missing / all TSAs unreachable
"""
from __future__ import annotations

import hashlib
import os
import time
import urllib.request
from pathlib import Path
from typing import Any

TSAS = (
    "http://timestamp.digicert.com",
    "http://timestamp.sectigo.com",
    "https://freetsa.org/tsr",
)

_STAMP_TIMEOUT = 10.0  # per TSA; sealing must not hang a robot


def _tsp():
    """Import the asn1crypto modules, or None if not installed."""
    try:
        from asn1crypto import tsp, algos, core
        return {"tsp": tsp, "algos": algos, "core": core}
    except ImportError:
        return None


def available() -> bool:
    return _tsp() is not None


def _build_request(mods, digest: bytes, nonce: int) -> bytes:
    return mods["tsp"].TimeStampReq({
        "version": "v1",
        "message_imprint": mods["tsp"].MessageImprint({
            "hash_algorithm": mods["algos"].DigestAlgorithm({"algorithm": "sha256"}),
            "hashed_message": digest,
        }),
        "nonce": nonce,
        "cert_req": True,  # embed the TSA cert chain so the .tsr verifies standalone
    }).dump()


def _parse_tst_info(mods, resp) -> Any:
    """TSTInfo out of a TimeStampResp; raises if the response is not granted."""
    status = resp["status"]["status"].native
    if status not in ("granted", "granted_with_mods"):
        raise ValueError(f"TSA refused: {status}")
    signed = resp["time_stamp_token"]["content"]
    return signed["encap_content_info"]["content"].parsed


def stamp_digest(digest: bytes, tsas: tuple[str, ...] = TSAS) -> bytes | None:
    """Submit a 32-byte SHA-256 digest to RFC 3161 TSAs; first grant wins.

    Returns the DER TimeStampResp (.tsr) bytes, or None when the library is
    missing or every TSA failed (offline robot — anchor opportunistically
    later via `upgrade`).
    """
    mods = _tsp()
    if mods is None or len(digest) != 32:
        return None
    nonce = int.from_bytes(os.urandom(8), "big")
    body = _build_request(mods, digest, nonce)
    for url in tsas:
        try:
            req = urllib.request.Request(
                url, data=body,
                headers={"Content-Type": "application/timestamp-query"})
            with urllib.request.urlopen(req, timeout=_STAMP_TIMEOUT) as r:
                raw = r.read()
            resp = mods["tsp"].TimeStampResp.load(raw)
            tst = _parse_tst_info(mods, resp)
            if tst["message_imprint"]["hashed_message"].native != digest:
                continue  # TSA answered for the wrong digest
            got_nonce = tst["nonce"].native
            if got_nonce is not None and got_nonce != nonce:
                continue  # replayed response
            return raw
        except Exception:
            continue
    return None


def stamp_file(path: str | Path, tsr_path: str | Path | None = None) -> dict[str, Any]:
    """Stamp a file's SHA-256; write `<path>.tsr` next to it on success."""
    path = Path(path)
    digest = hashlib.sha256(path.read_bytes()).digest()
    tsr_bytes = stamp_digest(digest)
    out = Path(tsr_path) if tsr_path else path.with_suffix(path.suffix + ".tsr")
    if tsr_bytes is None:
        return {"status": "unanchored", "digest": digest.hex(),
                "reason": "asn1crypto unavailable or all TSAs unreachable"}
    out.write_bytes(tsr_bytes)
    return {**status(out, expected_digest=digest), "digest": digest.hex(),
            "stamped_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}


def status(tsr_path: str | Path, expected_digest: bytes | None = None) -> dict[str, Any]:
    """Inspect a .tsr file: anchored / unanchored (+ details).

    Verifies digest binding when expected_digest is given. Offline — the
    token is self-contained; full signature-chain verification is one
    `openssl ts -verify` away and the proof file is standard DER.
    """
    mods = _tsp()
    path = Path(tsr_path)
    if not path.exists():
        return {"status": "unanchored", "reason": "no .tsr file"}
    if mods is None:
        return {"status": "unanchored",
                "reason": "asn1crypto not installed; cannot inspect", "tsr": str(path)}
    try:
        resp = mods["tsp"].TimeStampResp.load(path.read_bytes())
        tst = _parse_tst_info(mods, resp)
    except Exception as exc:
        return {"status": "unanchored", "reason": f"corrupt .tsr: {exc}"}
    if expected_digest is not None and \
            tst["message_imprint"]["hashed_message"].native != expected_digest:
        return {"status": "unanchored",
                "reason": "tsr digest does not match the seal: proof is for something else"}
    gen_time = tst["gen_time"].native
    out: dict[str, Any] = {
        "status": "anchored",
        "tsa_time": gen_time.strftime("%Y-%m-%dT%H:%M:%SZ"),
        "serial": str(tst["serial_number"].native),
        "policy": tst["policy"].native,
        "tsr": str(path),
    }
    tsa = tst["tsa"]
    if tsa.native is not None:
        try:
            out["tsa"] = tsa.chosen.chosen.native["common_name"]
        except Exception:
            pass
    return out


def upgrade(tsr_path: str | Path, digest: bytes | None = None) -> dict[str, Any]:
    """Anchor a seal that closed offline: stamp now if no valid .tsr exists.

    RFC 3161 is synchronous, so there is nothing to poll — "upgrade" means
    re-attempting the stamp with the seal's digest when connectivity returns.
    """
    path = Path(tsr_path)
    current = status(path, expected_digest=digest)
    if current["status"] == "anchored" or digest is None:
        return current
    tsr_bytes = stamp_digest(digest)
    if tsr_bytes is None:
        return {"status": "unanchored",
                "reason": "asn1crypto unavailable or all TSAs unreachable"}
    path.write_bytes(tsr_bytes)
    return status(path, expected_digest=digest)
