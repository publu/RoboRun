"""Integrity primitives: canonical hashing, Merkle root, Ed25519 identity.

This is a library, not a product surface. The black box is the MCAP
recorder (roborun/recorder.py) — it builds its chunk hash chain, O(1)
seal, and verify on these primitives. The event bus (roborun/events.py)
uses the same hashing to chain its live journal, and cross-robot beacons
(roborun/beacons.py) sign with the same identity key.

Stdlib-only core; Ed25519 signing activates if `cryptography` is
installed ([crypto] extra). One identity key per robot at
~/.roborun/ed25519.key, created on first use.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any

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


def sign_message(message: bytes) -> dict[str, str] | None:
    """Sign arbitrary bytes with this robot's Ed25519 identity key.

    The key is created on first use at ~/.roborun/ed25519.key and is the
    same identity used for run seals and cross-robot beacons. Returns None
    when `cryptography` is not installed.
    """
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
    sig = key.sign(message)
    pub = key.public_key().public_bytes(
        serialization.Encoding.Raw, serialization.PublicFormat.Raw)
    return {"algo": "ed25519", "signature": sig.hex(), "public_key": pub.hex()}


def verify_message(sig: dict | None, message: bytes) -> bool | None:
    """True/False if checkable, None if unsigned or cryptography missing."""
    if not sig:
        return None
    try:
        from cryptography.hazmat.primitives.asymmetric.ed25519 import (
            Ed25519PublicKey,
        )
    except ImportError:
        return None
    try:
        Ed25519PublicKey.from_public_bytes(
            bytes.fromhex(sig["public_key"])).verify(
            bytes.fromhex(sig["signature"]), message)
        return True
    except Exception:
        return False


def public_key_hex() -> str | None:
    """This robot's Ed25519 public key (creates the key on first call)."""
    signed = sign_message(b"roborun-identity")
    return signed["public_key"] if signed else None
